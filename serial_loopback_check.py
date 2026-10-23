#!/usr/bin/env python3
"""Check a USB-serial adapter's loopback plug from Linux, macOS or Windows:
modem control lines and TX->RX data. Standard library only (termios/fcntl on
Linux and macOS, ctypes/winreg on Windows), so it runs on a bare host with no
pyserial. The host is detected automatically.

wiring the xp5_sertest tests assumes:

    TX  -> RX
    RTS -> CTS
    DTR -> DSR + DCD + RI

Five parts:
  0. Driver : which driver ownn the port and the USB ids behind it. 
     Linux reads sysfs; macOS walks the ioreg tree from the port's IOSerialBSDClient 
     up to its driver and USB device, lists any other drivers matched to the same interface. 
     Windows finds the port's PnP instance in the registry plus the DCB the driver hands out.
  1. Line map. RTS and DTR are driven through all four combinations; CTS, DSR,
     DCD and RI are read back each time. The result is printed as a table plus
     "which input follows which output", so a plug wired differently shows up
     as what it IS, not just a FAIL. Then judged against --expect.
  2. Data. Every byte value 0x00-0xFF plus a pseudo-random block is sent at each
     --baud rate (8N1, no flow control) and read back; mismatches, missing
     bytes and extras are reported. Sized to about --data-secs of line time per
     rate (at least 32 bytes), so 110 baud doesn't take a minute and a half.
  3. Timing, per --baud rate:
       rtt   16 stop-and-wait single bytes (SERTEST P3's order, after one
             uncounted warm-up byte), in ms and in character times (8N1 = 10
             bits). The plug itself adds nothing, so this is the adapter and
             driver alone: what 86Box's passthrough sits on top of.
       rate  bytes sent back to back for about --rate-secs; the bits that
             arrive after the first read, over the time to the last read, is
             the rate the line actually runs at. More than --rate-tol percent
             off nominal FAILs: the driver accepted the rate but isn't using it.
       set   the speed the driver reads back after setting it.
  4. Break. A break is held on TX for each --break-hold time (alone, then
     followed by one data byte). Each trial is classed the way 86Box's
     serial_passthrough would see it (NDIAGS's EXTERNAL "Line Status" test
     needs LSR BI):
       A  break reported                           -- 86Box would set BI
       B  (Windows) CE_BREAK with an empty queue   -- 86Box drops it
       C  plain 0x00 byte(s), no break report      -- zeros arrive, break not flagged
       D  nothing                                  -- break not sent or not detected
     Linux/macOS read RX with PARMRK, so a detected break arrives as the marker
     0xFF 0x00 0x00 (the only thing char_serial_unescape turns into BI). Windows
     polls ClearCommError like char_serial_read does: CE_BREAK counts only when
     bytes are queued in that same call. Some drivers (pl2303) only flag a break
     on the NEXT received data, so a "D alone, A with the following byte" result
     is the driver, not the plug.

Usage:
    serial_loopback_check.py [DEVICE] [--baud 110,150,...,115200] [--no-ri]
                             [--expect rts-cts,dtr-dsr,dtr-dcd,dtr-ri]
                             [--settle 0.05] [--skip-data] [--skip-lines]
                             [--skip-timing] [--rate-secs 2] [--rate-tol 2]
                             [--break-baud 1200,9600,115200]
                             [--break-hold 10,100] [--skip-break] [--skip-driver]

--baud (data and timing) defaults to every SERTEST rate, 110-115200;
--break-baud is separate and keeps the old three.

DEVICE defaults to /dev/ttyUSB0 on Linux, the first /dev/cu.usb* on macOS (use
cu.*, not tty.*), or the first COM port the registry lists on Windows (needs a
native Windows Python, not MSYS2's /usr/bin/python). Needs read/write on it
(group dialout, or sudo). On Windows the port must be free: close 86Box or
anything else holding it. Exit status 0 = everything expected passed, 1 = a
check failed, 2 = could not open or drive the port.

NOTE: a plain USB-serial adapter has no loopback plug of its own -- you must
fit one (or jumper the pins) first, or every check will report "no".
"""
import argparse
import ctypes
import glob
import os
import platform
import random
import re
import struct
import subprocess
import sys
import threading
import time

try:
    import fcntl
    import select
    import termios
except ImportError:             # Windows
    fcntl = select = termios = None

DARWIN = sys.platform == "darwin"
WIN = sys.platform == "win32"

# Fallbacks if Python's termios doesn't export them: macOS _IO('t', 123/122),
# else the asm-generic Linux values (x86, arm, arm64).
TIOCSBRK = getattr(termios, "TIOCSBRK", 0x2000747B if DARWIN else 0x5427)
TIOCCBRK = getattr(termios, "TIOCCBRK", 0x2000747A if DARWIN else 0x5428)

INPUTS = ("CTS", "DSR", "DCD", "RI")
OUTPUTS = ("RTS", "DTR")
DEFAULT_EXPECT = "rts-cts,dtr-dsr,dtr-dcd,dtr-ri"
# SERTEST's page 2/3 rates (sertest.c baud_tab)
SERTEST_BAUDS = "110,150,300,600,1200,2400,4800,9600,19200,38400,57600,115200"
# SERTEST P3's stop-and-wait order; one 0x55 goes first, uncounted
RTT_BYTES = bytes([0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x01, 0x80]) * 2
RATES = (110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600,
         115200, 230400, 460800, 921600)


class PortError(Exception):
    """The port could not be opened, configured or driven."""


def _baud_table():
    """rate -> speed value for the backend. Linux uses the B<rate> constants
    (Python's termios lacks the rates above 230400 on macOS). On macOS speed_t
    IS the rate in bits/s, so the plain integer is valid for any rate the
    driver accepts. Windows takes the rate itself."""
    table = {}
    for rate in RATES:
        const = getattr(termios, f"B{rate}", None)
        if WIN:
            table[rate] = rate
        elif const is not None:
            table[rate] = const
        elif DARWIN:
            table[rate] = rate
    return table


BAUD = _baud_table()


def default_device():
    """First USB-serial node: /dev/ttyUSB0 on Linux; on macOS the call-out
    (cu.*) node, since tty.* blocks open() waiting for carrier; on Windows the
    first COM port in SERIALCOMM, preferring adapters over the built-in
    \\Device\\SerialN ports."""
    if WIN:
        try:
            import winreg
            ports = []
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as k:
                i = 0
                while True:
                    try:
                        name, value, _ = winreg.EnumValue(k, i)
                    except OSError:
                        break
                    ports.append((name, value))
                    i += 1
            ports.sort(key=lambda p: (p[0].lower().startswith("\\device\\serial"), p[1]))
            if ports:
                return ports[0][1]
        except OSError:
            pass
        return "COM3"
    if DARWIN:
        found = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")
                       + glob.glob("/dev/cu.PL2303*") + glob.glob("/dev/cu.wchusbserial*"))
        if found:
            return found[0]
    return "/dev/ttyUSB0"


# --------------------------------------------------------------------------
# Port backends. Everything below the checks talks to this interface:
#   configure(baud, parmrk)  set_outputs(rts, dtr)  lines() -> (inputs, note)
#   flush()  drain_tx()  speed()  write_all(data)  wait_readable(t)  read()
#   break_trial(hold_s, window_s, follow) -> (t_off_ms, log, case, why, note)
# --------------------------------------------------------------------------

class PosixPort:
    concurrent_io = True        # a writer thread may run beside the reader

    def __init__(self, path):
        try:
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise PortError(f"cannot open {path}: {e}\n"
                            "  (is the adapter attached to THIS host and not passed through to a VM? "
                            "need group dialout or sudo?)")

    def close(self):
        os.close(self.fd)

    def driver_rows(self):
        return []

    def configure(self, baud, parmrk=False):
        """Raw 8N1, receiver on, modem lines ignored, no flow control of any kind
        (hardware flow control would gate TX on CTS and mask a miswired plug).
        parmrk makes a received break read as 0xFF 0x00 0x00."""
        try:
            attrs = termios.tcgetattr(self.fd)
            _, oflag, cflag, lflag, _, _, cc = attrs
            oflag = 0
            lflag = 0
            cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
            cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            speed = BAUD[baud]
            iflag = termios.PARMRK if parmrk else 0
            termios.tcsetattr(self.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error as e:
            raise PortError(f"tcsetattr refused the rate: {e}")

    def set_outputs(self, rts, dtr):
        """Drive RTS and DTR to exactly the requested levels."""
        try:
            for bit, on in ((termios.TIOCM_RTS, rts), (termios.TIOCM_DTR, dtr)):
                req = termios.TIOCMBIS if on else termios.TIOCMBIC
                fcntl.ioctl(self.fd, req, struct.pack("I", bit))
        except OSError as e:
            raise PortError(str(e))

    def lines(self):
        try:
            state = struct.unpack("I", fcntl.ioctl(self.fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
        except OSError as e:
            raise PortError(str(e))
        bits = {"CTS": termios.TIOCM_CTS, "DSR": termios.TIOCM_DSR,
                "DCD": termios.TIOCM_CAR, "RI": termios.TIOCM_RNG}
        inputs = {n: bool(state & b) for n, b in bits.items()}
        note = (f"TIOCMGET {state:#06x}, readback RTS={int(bool(state & termios.TIOCM_RTS))} "
                f"DTR={int(bool(state & termios.TIOCM_DTR))}")
        return inputs, note

    def flush(self):
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def drain_tx(self):
        termios.tcdrain(self.fd)

    def speed(self):
        """The output speed tcgetattr() reports, as bits/s."""
        speed = termios.tcgetattr(self.fd)[5]
        for rate, const in BAUD.items():
            if const == speed:
                return rate
        return speed

    def write_all(self, data):
        """Write every byte to the non-blocking fd, waiting for the tty output
        queue to drain when it fills (macOS's is small, so os.write raises
        EAGAIN or short-writes at low baud rates)."""
        while data:
            try:
                n = os.write(self.fd, data)
            except BlockingIOError:
                select.select([], [self.fd], [], 1.0)
                continue
            data = data[n:]

    def wait_readable(self, timeout):
        r, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        return bool(r)

    def read(self, n=4096):
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b""

    def _send_break(self, hold, log):
        """Hold a break for `hold` seconds, reading RX meanwhile. Falls back to
        tcsendbreak (a fixed 0.25-0.5 s) if the driver rejects TIOCSBRK."""
        t0 = time.monotonic()
        try:
            fcntl.ioctl(self.fd, TIOCSBRK)
        except OSError:
            termios.tcsendbreak(self.fd, 0)
            return t0, "tcsendbreak"
        try:
            log += read_timed(self, t0, t0 + hold, "held")
        finally:
            fcntl.ioctl(self.fd, TIOCCBRK)
        return t0, None

    def break_trial(self, hold, window, follow):
        self.flush()
        raw = []
        t0, fallback = self._send_break(hold, raw)
        t_off = (time.monotonic() - t0) * 1000.0
        if follow is not None:
            self.write_all(bytes([follow]))
        raw += read_timed(self, t0, time.monotonic() + window, "after")
        breaks, nuls, other = decode_parmrk(b"".join(c for _, _, c in raw))
        if breaks:
            case, why = "A", f"{breaks} break marker(s) -- 86Box would set BI"
            if follow is not None and follow not in other:
                why += f"; the following {follow:#04x} was consumed by the break flag"
        elif nuls:
            case, why = "C", f"{nuls} plain 0x00 byte(s), no break marker"
        else:
            case, why = "D", "no break marker and no 0x00 -- break not sent or not detected"
        note = f" (TIOCSBRK refused, used {fallback})" if fallback else ""
        log = [(ms, phase, f"read {chunk.hex(' ')}") for ms, phase, chunk in raw]
        return t_off, log, case, why, note


DWORD = ctypes.c_uint32
WORD = ctypes.c_uint16
BYTE = ctypes.c_uint8

CE_FLAGS = ((0x0001, "RXOVER"), (0x0002, "OVERRUN"), (0x0004, "PARITY"),
            (0x0008, "FRAME"), (0x0010, "BREAK"))
CE_BREAK = 0x0010


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", DWORD),
        ("BaudRate", DWORD),
        ("fBinary", DWORD, 1),
        ("fParity", DWORD, 1),
        ("fOutxCtsFlow", DWORD, 1),
        ("fOutxDsrFlow", DWORD, 1),
        ("fDtrControl", DWORD, 2),
        ("fDsrSensitivity", DWORD, 1),
        ("fTXContinueOnXoff", DWORD, 1),
        ("fOutX", DWORD, 1),
        ("fInX", DWORD, 1),
        ("fErrorChar", DWORD, 1),
        ("fNull", DWORD, 1),
        ("fRtsControl", DWORD, 2),
        ("fAbortOnError", DWORD, 1),
        ("fDummy2", DWORD, 17),
        ("wReserved", WORD),
        ("XonLim", WORD),
        ("XoffLim", WORD),
        ("ByteSize", BYTE),
        ("Parity", BYTE),
        ("StopBits", BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", WORD),
    ]


class COMSTAT(ctypes.Structure):
    _fields_ = [("fFlags", DWORD), ("cbInQue", DWORD), ("cbOutQue", DWORD)]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [("ReadIntervalTimeout", DWORD), ("ReadTotalTimeoutMultiplier", DWORD),
                ("ReadTotalTimeoutConstant", DWORD), ("WriteTotalTimeoutMultiplier", DWORD),
                ("WriteTotalTimeoutConstant", DWORD)]


def ce_text(err):
    names = [n for bit, n in CE_FLAGS if err & bit]
    return "|".join(names) if names else "-"


class WinPort:
    """Win32 comm API. One synchronous handle: Windows serializes synchronous
    I/O per handle, so there is no writer thread (concurrent_io False); the
    rate test interleaves write_some() with reads instead."""
    concurrent_io = False

    def __init__(self, name):
        if not hasattr(ctypes, "WinDLL"):
            raise PortError("needs a native Windows Python (MSYS2's /usr/bin/python has no "
                            "ctypes.WinDLL)")
        k = self.k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k.CreateFileW.restype = ctypes.c_void_p
        k.CreateFileW.argtypes = [ctypes.c_wchar_p, DWORD, DWORD, ctypes.c_void_p,
                                  DWORD, DWORD, ctypes.c_void_p]
        for fn in ("CloseHandle", "SetCommBreak", "ClearCommBreak"):
            getattr(k, fn).argtypes = [ctypes.c_void_p]
        k.GetCommState.argtypes = [ctypes.c_void_p, ctypes.POINTER(DCB)]
        k.SetCommState.argtypes = [ctypes.c_void_p, ctypes.POINTER(DCB)]
        k.SetCommTimeouts.argtypes = [ctypes.c_void_p, ctypes.POINTER(COMMTIMEOUTS)]
        k.SetupComm.argtypes = [ctypes.c_void_p, DWORD, DWORD]
        k.PurgeComm.argtypes = [ctypes.c_void_p, DWORD]
        k.EscapeCommFunction.argtypes = [ctypes.c_void_p, DWORD]
        k.GetCommModemStatus.argtypes = [ctypes.c_void_p, ctypes.POINTER(DWORD)]
        k.ClearCommError.argtypes = [ctypes.c_void_p, ctypes.POINTER(DWORD), ctypes.POINTER(COMSTAT)]
        k.ReadFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p, DWORD, ctypes.POINTER(DWORD), ctypes.c_void_p]
        k.WriteFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p, DWORD, ctypes.POINTER(DWORD), ctypes.c_void_p]
        path = name if name.startswith("\\\\.\\") else "\\\\.\\" + name
        h = k.CreateFileW(path, 0x80000000 | 0x40000000, 0, None, 3, 0, None)
        if h is None or h == ctypes.c_void_p(-1).value:
            err = ctypes.get_last_error()
            hint = ""
            if err == 5:
                hint = " -- port is in use; close 86Box or whatever else holds it"
            elif err == 2:
                hint = " -- no such port; check Device Manager for the COM number"
            raise PortError("cannot open %s: %s (%d)%s" % (path, ctypes.FormatError(err).strip(), err, hint))
        self.h = h
        k.SetupComm(h, 2048, 2048)

    def _err(self, what):
        err = ctypes.get_last_error()
        return PortError("%s failed: %s (%d)" % (what, ctypes.FormatError(err).strip(), err))

    def close(self):
        self.k32.CloseHandle(self.h)

    def _dcb(self):
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not self.k32.GetCommState(self.h, ctypes.byref(dcb)):
            raise self._err("GetCommState")
        return dcb

    def driver_rows(self):
        """The DCB 86Box inherits (it copies GetCommState and only overrides some
        fields). fNull=1 would make the driver discard every received NUL --
        including the 0x00 a break produces. Read before configure() resets it."""
        try:
            d = self._dcb()
        except PortError as e:
            return [("dcb", str(e))]
        rows = [("dcb", "baud %d  fNull %d  fErrorChar %d (0x%02x)  fAbortOnError %d  fOutxCtsFlow %d  "
                        "fOutxDsrFlow %d  fDsrSensitivity %d"
                 % (d.BaudRate, d.fNull, d.fErrorChar, ord(d.ErrorChar or b"\0"), d.fAbortOnError,
                    d.fOutxCtsFlow, d.fOutxDsrFlow, d.fDsrSensitivity))]
        if d.fNull:
            rows.append(("note", "fNull=1 -- 86Box inherits this, so the driver drops received NULs"))
        return rows

    def configure(self, baud, parmrk=False):
        """8N1, no flow control, DTR/RTS enabled, fNull off -- what 86Box's
        char_serial_port_config applies (plus fNull, forced off here so a NUL
        from a break can't be hidden by the driver's default). parmrk is
        ignored: Windows reports a break through ClearCommError instead."""
        dcb = self._dcb()
        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.fBinary = 1
        dcb.fParity = 1
        dcb.fOutxCtsFlow = 0
        dcb.fOutxDsrFlow = 0
        dcb.fDtrControl = 1
        dcb.fRtsControl = 1
        dcb.fDsrSensitivity = 0
        dcb.fOutX = 0
        dcb.fInX = 0
        dcb.fErrorChar = 0
        dcb.fNull = 0
        dcb.fAbortOnError = 0
        if not self.k32.SetCommState(self.h, ctypes.byref(dcb)):
            raise self._err("SetCommState")
        # Reads return at once with whatever is queued; writes block, no timeout.
        to = COMMTIMEOUTS(0xFFFFFFFF, 0, 0, 0, 0)
        if not self.k32.SetCommTimeouts(self.h, ctypes.byref(to)):
            raise self._err("SetCommTimeouts")
        self.flush()

    def set_outputs(self, rts, dtr):
        self.k32.EscapeCommFunction(self.h, 3 if rts else 4)    # SETRTS / CLRRTS
        self.k32.EscapeCommFunction(self.h, 5 if dtr else 6)    # SETDTR / CLRDTR

    def lines(self):
        st = DWORD(0)
        if not self.k32.GetCommModemStatus(self.h, ctypes.byref(st)):
            raise self._err("GetCommModemStatus")
        v = st.value
        inputs = {"CTS": bool(v & 0x10), "DSR": bool(v & 0x20), "RI": bool(v & 0x40), "DCD": bool(v & 0x80)}
        return inputs, f"GetCommModemStatus {v:#06x}, RTS/DTR not readable"

    def flush(self):
        self.k32.PurgeComm(self.h, 0x0004 | 0x0008)             # TXCLEAR | RXCLEAR

    def _poll(self):
        """One 86Box-style receive poll: ClearCommError, then ReadFile only if
        bytes are queued. Returns (errors, bytes_queued, data_read, bytes_out)."""
        err = DWORD(0)
        st = COMSTAT()
        if not self.k32.ClearCommError(self.h, ctypes.byref(err), ctypes.byref(st)):
            raise self._err("ClearCommError")
        data = b""
        if st.cbInQue:
            buf = ctypes.create_string_buffer(st.cbInQue)
            got = DWORD(0)
            if not self.k32.ReadFile(self.h, buf, st.cbInQue, ctypes.byref(got), None):
                raise self._err("ReadFile")
            data = buf.raw[:got.value]
        return err.value, st.cbInQue, data, st.cbOutQue

    def drain_tx(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            err = DWORD(0)
            st = COMSTAT()
            self.k32.ClearCommError(self.h, ctypes.byref(err), ctypes.byref(st))
            if not st.cbOutQue:
                return
            time.sleep(0.001)

    def speed(self):
        return self._dcb().BaudRate

    def write_all(self, data):
        while data:
            n = DWORD(0)
            if not self.k32.WriteFile(self.h, data, len(data), ctypes.byref(n), None):
                raise self._err("WriteFile")
            data = data[n.value:]

    def write_some(self, data):
        """Queue up to 128 bytes if the driver's TX queue has room; returns how
        many were taken (0 = full, try again)."""
        err = DWORD(0)
        st = COMSTAT()
        self.k32.ClearCommError(self.h, ctypes.byref(err), ctypes.byref(st))
        if st.cbOutQue > 512:
            return 0
        chunk = data[:128]
        n = DWORD(0)
        if not self.k32.WriteFile(self.h, chunk, len(chunk), ctypes.byref(n), None):
            raise self._err("WriteFile")
        return n.value

    def wait_readable(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            err = DWORD(0)
            st = COMSTAT()
            if not self.k32.ClearCommError(self.h, ctypes.byref(err), ctypes.byref(st)):
                raise self._err("ClearCommError")
            if st.cbInQue:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.0003)

    def read(self, n=4096):
        err = DWORD(0)
        st = COMSTAT()
        if not self.k32.ClearCommError(self.h, ctypes.byref(err), ctypes.byref(st)):
            raise self._err("ClearCommError")
        if not st.cbInQue:
            return b""
        m = min(n, st.cbInQue)
        buf = ctypes.create_string_buffer(m)
        got = DWORD(0)
        if not self.k32.ReadFile(self.h, buf, m, ctypes.byref(got), None):
            raise self._err("ReadFile")
        return buf.raw[:got.value]

    def _reset(self):
        self.flush()
        time.sleep(0.05)
        self._poll()
        self._poll()

    def _poll_until(self, t0, deadline, log, phase):
        """Poll as fast as possible until perf_counter() >= deadline, logging
        every poll that saw an error flag or data."""
        while time.perf_counter() < deadline:
            err, inq, data, _ = self._poll()
            if err or data:
                log.append(((time.perf_counter() - t0) * 1000.0, phase, err, inq, data))
            time.sleep(0)

    def break_trial(self, hold, window, follow):
        self._reset()
        raw = []
        t0 = time.perf_counter()
        if not self.k32.SetCommBreak(self.h):
            raise self._err("SetCommBreak")
        self._poll_until(t0, t0 + hold, raw, "held")
        if not self.k32.ClearCommBreak(self.h):
            raise self._err("ClearCommBreak")
        t_off = (time.perf_counter() - t0) * 1000.0
        if follow is not None:
            self.write_all(bytes([follow]))
        self._poll_until(t0, time.perf_counter() + window, raw, "after")
        brk = [e for e in raw if e[2] & CE_BREAK]
        nuls = sum(e[4].count(0) for e in raw)
        if [e for e in brk if e[3] > 0]:
            case, why = "A", "CE_BREAK arrived with bytes queued -- 86Box would set LSR BI"
        elif brk:
            case, why = "B", ("CE_BREAK only with an EMPTY queue -- 86Box's char_serial_read "
                              "clears it and never sets BI")
        elif nuls:
            case, why = "C", (f"{nuls} NUL byte(s) but no CE_BREAK -- driver passes break as plain "
                              "data; 86Box sees 0x00, never BI")
        else:
            case, why = "D", "no CE_BREAK and no data -- break not sent or not detected"
        log = [(ms, phase, "err=%-14s inq=%-3d read=%s" % (ce_text(err), inq, data.hex(" ") if data else "-"))
               for ms, phase, err, inq, data in raw]
        return t_off, log, case, why, ""


def open_port(device):
    return WinPort(device) if WIN else PosixPort(device)


# --------------------------------------------------------------------------
# Driver info (informational)
# --------------------------------------------------------------------------

def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def driver_info_linux(dev):
    """(label, value) rows from sysfs: every driver bound on the way up from the
    tty to its USB device, the USB ids, and the FTDI latency timer if present."""
    name = os.path.basename(os.path.realpath(dev))
    node = os.path.join("/sys/class/tty", name, "device")
    rows = [("kernel", os.uname().release)]
    if not os.path.exists(node):
        rows.append(("driver", f"none ({name} has no sysfs device: pty or virtual port)"))
        return rows
    path = os.path.realpath(node)
    latency = None
    while path.startswith("/sys/devices") and path != "/sys/devices":
        drv = os.path.join(path, "driver")
        if os.path.islink(drv):
            mod = os.path.basename(os.path.realpath(os.path.join(drv, "module")))
            via = os.path.basename(os.path.realpath(drv))
            rows.append(("bound", f"{os.path.basename(path)} -> {via}"
                         + (f" (module {mod})" if mod != via and os.path.isdir(os.path.join(drv, "module")) else "")))
        if latency is None:
            latency = _read(os.path.join(path, "latency_timer"))
        if os.path.exists(os.path.join(path, "idVendor")):
            vid, pid = _read(os.path.join(path, "idVendor")), _read(os.path.join(path, "idProduct"))
            rows.append(("usb id", f"{vid}:{pid}  bcdDevice {_read(os.path.join(path, 'bcdDevice'))}"))
            for label, f in (("vendor", "manufacturer"), ("product", "product"), ("serial", "serial")):
                v = _read(os.path.join(path, f))
                if v:
                    rows.append((label, v))
            break
        path = os.path.dirname(path)
    if latency is not None:
        rows.append(("latency", f"{latency} ms (FTDI latency_timer)"))
    return rows


def _ioreg_nodes(text):
    """Parse `ioreg -l` text into [{depth, name, cls, props}]. A node line has
    '+-o'; its depth is that column. Every following line up to the next node
    line is one of its properties."""
    nodes = []
    for line in text.splitlines():
        i = line.find("+-o ")
        if i >= 0:
            m = re.match(r"\+-o (.*?)\s+<class (\w+)", line[i:])
            nodes.append({"depth": i, "name": m.group(1) if m else line[i + 4:].strip(),
                          "cls": m.group(2) if m else "?", "props": []})
        elif nodes:
            nodes[-1]["props"].append(line)
    return nodes


def _prop(node, key):
    """Value of a simple '"key" = value' property, quotes stripped."""
    for line in node["props"]:
        m = re.search(r'"%s" = (.+)$' % re.escape(key), line)
        if m:
            return m.group(1).strip().strip('"')
    return None


def _personality(node):
    """(bundle id, IOUserClass, IOUserServerName) from IOMatchedPersonality."""
    for line in node["props"]:
        if '"IOMatchedPersonality"' in line:
            f = lambda k: (re.search(r'"%s"="([^"]+)"' % k, line) or [None, None])[1]
            return f("CFBundleIdentifier"), f("IOUserClass"), f("IOUserServerName")
    return None, None, None


def driver_info_macos(dev):
    """(label, value) rows: ioreg ancestors of the port's IOSerialBSDClient."""
    rows = [("macOS", os.popen("sw_vers -productVersion").read().strip() or "?")]
    out = subprocess.run(["ioreg", "-l", "-w0"], capture_output=True, text=True, errors="replace", timeout=30).stdout
    nodes = _ioreg_nodes(out)
    want = os.path.realpath(dev)
    bsd = None
    for i, n in enumerate(nodes):
        if n["cls"] == "IOSerialBSDClient" and any(
                f'"{want}"' in l and ("IOCalloutDevice" in l or "IOTTYDevice" in l) for l in n["props"]):
            bsd = i
            break
    if bsd is None:
        rows.append(("driver", f"{want} not found in the ioreg tree"))
        return rows

    def parent(i):
        for j in range(i - 1, -1, -1):
            if nodes[j]["depth"] < nodes[i]["depth"]:
                return j
        return None

    owner = parent(bsd)
    iface = parent(owner) if owner is not None else None
    if owner is not None:
        o = nodes[owner]
        bundle, uclass, server = _personality(o)
        rows.append(("driver", f"{o['name']} (class {o['cls']})"))
        for label, v in (("bundle", bundle), ("user class", uclass), ("dext server", server)):
            if v:
                rows.append((label, v))
        if bundle:
            ext = subprocess.run(["systemextensionsctl", "list"], capture_output=True, text=True, errors="replace", timeout=15)
            for l in (ext.stdout + ext.stderr).splitlines():
                if bundle in l:
                    rows.append(("sysext", " ".join(l.split())))
    if iface is not None:
        step = nodes[owner]["depth"] - nodes[iface]["depth"]
        rows.append(("interface", f"{nodes[iface]['name']} (class {nodes[iface]['cls']})"))
        for k in range(iface + 1, len(nodes)):
            if nodes[k]["depth"] <= nodes[iface]["depth"]:
                break
            if nodes[k]["depth"] == nodes[iface]["depth"] + step and k != owner:
                b, _, _ = _personality(nodes[k])
                rows.append(("also matched", f"{nodes[k]['name']}" + (f" ({b})" if b else "")))
    j = iface
    while j is not None and nodes[j]["cls"] != "IOUSBHostDevice":
        j = parent(j)
    if j is not None:
        u = nodes[j]
        vid, pid = _prop(u, "idVendor"), _prop(u, "idProduct")
        if vid and pid:
            rows.append(("usb id", f"{int(vid):04x}:{int(pid):04x}  bcdDevice {_prop(u, 'bcdDevice')}"))
        for label, key in (("vendor", "USB Vendor Name"), ("product", "USB Product Name"),
                           ("serial", "USB Serial Number")):
            v = _prop(u, key)
            if v:
                rows.append((label, v))
    return rows


def driver_info_windows(dev):
    """(label, value) rows from the registry: the PnP instance whose
    'Device Parameters\\PortName' is this port (USB adapters, FTDIBUS, com0com,
    built-in UARTs alike), its service, and the class-key driver record."""
    import winreg
    port = dev.replace("\\\\.\\", "").upper()
    hklm = winreg.HKEY_LOCAL_MACHINE
    enum = r"SYSTEM\CurrentControlSet\Enum"
    rows = [("windows", platform.platform())]

    def subkeys(path):
        out = []
        try:
            with winreg.OpenKey(hklm, path) as k:
                i = 0
                while True:
                    try:
                        out.append(winreg.EnumKey(k, i))
                    except OSError:
                        break
                    i += 1
        except OSError:
            pass
        return out

    def value(path, name):
        try:
            with winreg.OpenKey(hklm, path) as k:
                return winreg.QueryValueEx(k, name)[0]
        except OSError:
            return None

    found = None
    for bus in subkeys(enum):
        for dev_id in subkeys(f"{enum}\\{bus}"):
            for inst in subkeys(f"{enum}\\{bus}\\{dev_id}"):
                base = f"{enum}\\{bus}\\{dev_id}\\{inst}"
                pn = value(base + r"\Device Parameters", "PortName")
                if pn and pn.upper() == port:
                    found = (bus, dev_id, inst, base)
                    break
            if found:
                break
        if found:
            break
    if not found:
        rows.append(("driver", f"{port} has no PortName under HKLM\\{enum}"))
        return rows
    bus, dev_id, inst, base = found
    rows.append(("instance", f"{bus}\\{dev_id}\\{inst}"))
    desc = value(base, "FriendlyName") or value(base, "DeviceDesc")
    if desc:
        rows.append(("device", str(desc).split(";")[-1]))
    hw = value(base, "HardwareID")
    hw = hw[0] if isinstance(hw, list) and hw else hw
    if hw:
        rows.append(("hardware id", hw))
        m = re.search(r"VID_([0-9A-Fa-f]{4})[&+]PID_([0-9A-Fa-f]{4})", hw)
        if m:
            rows.append(("usb id", f"{m.group(1).lower()}:{m.group(2).lower()}"))
    mfg = value(base, "Mfg")
    if mfg:
        rows.append(("vendor", str(mfg).split(";")[-1]))
    svc = value(base, "Service")
    if svc:
        rows.append(("service", svc))
    cls = value(base, "Driver")
    if cls:
        crec = r"SYSTEM\CurrentControlSet\Control\Class\%s" % cls
        for label, key in (("driver", "DriverDesc"), ("provider", "ProviderName"),
                           ("version", "DriverVersion"), ("date", "DriverDate"),
                           ("inf", "InfPath"), ("inf section", "InfSection")):
            v = value(crec, key)
            if v:
                rows.append((label, str(v)))
    lat = value(base + r"\Device Parameters", "LatencyTimer")
    if lat is not None:
        rows.append(("latency", f"{lat} ms (FTDI LatencyTimer)"))
    return rows


def show_driver(dev, port):
    """Informational; any failure is reported and never fails the run."""
    print("== Driver ==")
    try:
        if WIN:
            rows = driver_info_windows(dev)
        elif DARWIN:
            rows = driver_info_macos(dev)
        else:
            rows = driver_info_linux(dev)
        rows += port.driver_rows()
    except Exception as e:
        print(f"  driver info unavailable: {e}")
        return
    for label, value in rows:
        print(f"  {label + ':':<14}{value}")


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------

def read_all(port, want, timeout):
    """Collect up to `want` bytes, giving up once `timeout` seconds pass with
    no new data (or the total deadline of timeout + a generous per-byte cost)."""
    buf = bytearray()
    last = time.monotonic()
    while len(buf) < want and time.monotonic() - last < timeout:
        if port.wait_readable(0.05):
            chunk = port.read()
            if chunk:
                buf += chunk
                last = time.monotonic()
    # drain stragglers so "extra bytes" is reported rather than leaking into
    # the next pass
    time.sleep(0.05)
    buf += drain(port)
    return bytes(buf)


def check_lines(port, settle, expect, use_ri):
    print("== Modem control lines ==")
    print("  RTS DTR | CTS DSR DCD  RI")
    seen = {}      # (out_name, in_name) -> follows in every combo it could
    table = {}
    for rts in (0, 1):
        for dtr in (0, 1):
            port.set_outputs(rts, dtr)
            time.sleep(settle)
            inputs, note = port.lines()
            table[(rts, dtr)] = inputs
            row = "  ".join(" 1 " if inputs[n] else " 0 " for n in INPUTS)
            print(f"   {rts}   {dtr}  | {row}   [{note}]")
    port.set_outputs(0, 0)

    # An input "follows" an output if its level equals that output's level in
    # all four combinations (so it is independent of the other output).
    for oname in OUTPUTS:
        for iname in INPUTS:
            follows = all(
                table[(rts, dtr)][iname] == bool({"RTS": rts, "DTR": dtr}[oname])
                for rts in (0, 1) for dtr in (0, 1))
            seen[(oname.lower(), iname.lower())] = follows

    print("\n  Observed wiring (output -> inputs that follow it):")
    for oname in OUTPUTS:
        who = [i for i in INPUTS if seen[(oname.lower(), i.lower())]]
        print(f"    {oname} -> {', '.join(who) if who else '(nothing)'}")
    stuck = [i for i in INPUTS
             if len({table[k][i] for k in table}) == 1]
    stuck_hi = [i for i in stuck if table[(0, 0)][i]]
    if stuck_hi:
        print(f"  Inputs stuck HIGH regardless of RTS/DTR: {', '.join(stuck_hi)}")

    ok = True
    print("\n  Against expected wiring:")
    for pair in expect:
        o, i = pair.split("-")
        if i == "ri" and not use_ri:
            print(f"    {o.upper()} -> {i.upper():3} skipped (--no-ri)")
            continue
        good = seen.get((o, i), False)
        ok &= good
        print(f"    {o.upper()} -> {i.upper():3} {'PASS' if good else 'FAIL'}")
    # extra, unexpected links are a wiring problem too (e.g. RTS also on DSR)
    expected = set(tuple(p.split("-")) for p in expect)
    for (o, i), f in sorted(seen.items()):
        if f and (o, i) not in expected and not (i == "ri" and not use_ri):
            print(f"    {o.upper()} -> {i.upper():3} FAIL (unexpected link)")
            ok = False
    return ok


def char_ms(baud):
    """One 8N1 character (10 bits) in ms."""
    return 10000.0 / baud


def check_data(port, bauds, secs):
    print("\n== Data loopback (TX -> RX, 8N1, no flow control) ==")
    port.set_outputs(1, 1)           # some plugs/adapters want the lines up
    time.sleep(0.05)
    rng = random.Random(0x5E27)
    pattern = bytes(range(256)) + bytes(rng.randrange(256) for _ in range(768))
    all_ok = True
    for baud in bauds:
        try:
            port.configure(baud)
        except PortError as e:
            print(f"  {baud:>7} baud: FAIL ({e})")
            all_ok = False
            continue
        # about `secs` of line time: every byte value once the rate allows it
        n = max(32, min(len(pattern), int(baud * secs / 10)))
        want = pattern[:n] if n >= 256 else bytes(range(0, 256, 256 // n))[:n]
        # chunked so the adapter's FIFO/driver buffer is never overrun
        for i in range(0, len(want), 64):
            port.write_all(want[i:i + 64])
        port.drain_tx()
        got = read_all(port, len(want), timeout=1.0 + 11 * len(want) / baud / 4)
        if got == want:
            print(f"  {baud:>7} baud: PASS ({len(got)} bytes)")
            continue
        all_ok = False
        m = min(len(got), len(want))
        bad = [k for k in range(m) if got[k] != want[k]]
        if len(got) == 0:
            why = "nothing received -- TX is not wired to RX"
        else:
            why = (f"got {len(got)}/{len(want)} bytes, "
                   f"{len(bad)} wrong"
                   + (f", first at offset {bad[0]} (sent {want[bad[0]]:#04x} got {got[bad[0]]:#04x})" if bad else ""))
        print(f"  {baud:>7} baud: FAIL ({why})")
    port.set_outputs(0, 0)
    return all_ok


def drain(port):
    """Read and return whatever is already waiting, without blocking."""
    buf = bytearray()
    while port.wait_readable(0):
        chunk = port.read()
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def settle(port, baud):
    """After a rate change: flush, let the line sit idle for a few characters
    (a byte still on the wire, or a glitch from the switch, lands in that
    time), then drain. Returns what was drained."""
    port.flush()
    time.sleep(max(0.05, 3 * char_ms(baud) / 1000.0))
    return drain(port)


def rtt_one(port, byte, timeout):
    """Send one byte and wait for that byte to come back, without flushing
    (a flush right before the write can cost the byte on some drivers).
    Returns (ms or None, stray bytes: read before the write, or read while
    waiting that weren't the byte sent)."""
    strays = bytearray(drain(port))
    t0 = time.monotonic()
    port.write_all(bytes([byte]))
    deadline = t0 + timeout
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return None, bytes(strays)
        if port.wait_readable(left):
            ms = (time.monotonic() - t0) * 1000.0
            for b in port.read():
                if b == byte and ms is not None:
                    hit, ms = ms, None
                else:
                    strays.append(b)
            if ms is None:
                return hit, bytes(strays)


def measure_rate(port, baud, secs):
    """Send about `secs` of back-to-back bytes (from a writer thread, or on
    Windows interleaved with the reads) while timestamping every read. Returns
    (measured bits/s or None, bytes sent, bytes received, bytes wrong, ms
    first..last read)."""
    n = max(16, min(16384, int(baud * secs / 10)))
    data = bytes((0x55 + 7 * k) & 0xFF for k in range(n))
    drain(port)
    writer = None
    pending = data
    if port.concurrent_io:
        writer = threading.Thread(target=port.write_all, args=(data,), daemon=True)
        pending = b""
    reads = []                      # (t, nbytes)
    got = bytearray()
    if writer:
        writer.start()
    idle = max(0.5, 30 * char_ms(baud) / 1000.0)
    last = time.monotonic()
    while len(got) < n and time.monotonic() - last < idle:
        if pending:
            took = port.write_some(pending)
            pending = pending[took:]
            if took:
                last = time.monotonic()
        if port.wait_readable(0.01 if not pending else 0):
            chunk = port.read()
            if chunk:
                last = time.monotonic()
                reads.append((last, len(chunk)))
                got += chunk
    if writer:
        writer.join(5.0)
    got += drain(port)
    wrong = sum(1 for a, b in zip(got, data) if a != b) + abs(len(got) - n)
    why = ""
    if wrong:
        bad = next((k for k, (a, b) in enumerate(zip(got, data)) if a != b), min(len(got), n))
        why = f"first bad at {bad}: sent {data[bad:bad + 4].hex(' ')} got {bytes(got[bad:bad + 4]).hex(' ')}"
        for k in (1, 2, -1, -2):        # whole run off by a stray/lost byte?
            a, b = (got[k:], data) if k > 0 else (got, data[-k:])
            m = min(len(a), len(b))
            if m and a[:m] == b[:m]:
                why += f"; the rest matches shifted by {k:+d}"
                break
    if len(reads) < 2:
        return None, n, len(got), wrong, 0.0, why
    # bytes in the first read arrived before the clock started
    after = sum(c for _, c in reads) - reads[0][1]
    span = reads[-1][0] - reads[0][0]
    return after * 10 / span, n, len(got), wrong, span * 1000.0, why


def check_timing(port, bauds, secs, tol):
    print("\n== Timing (8N1, char = 10 bits) ==")
    print("  rtt: one byte at a time, ms and in char times; "
          "rate: back-to-back bytes, bits/s on the wire")
    port.set_outputs(1, 1)
    time.sleep(0.05)
    all_ok = True
    for baud in bauds:
        try:
            port.configure(baud)
        except PortError as e:
            print(f"  {baud:>7} baud: FAIL ({e})")
            all_ok = False
            continue
        cms = char_ms(baud)
        timeout = max(0.5, 30 * cms / 1000.0)
        notes = []
        junk = settle(port, baud)
        if junk:
            notes.append(f"after the rate switch: {junk[:16].hex(' ')}")
        ms, strays = rtt_one(port, 0x55, timeout)          # warm-up, uncounted
        if ms is None or strays:
            notes.append("warm-up 55: " + ("lost" if ms is None else f"{ms:.2f} ms")
                         + (f", strays {strays[:16].hex(' ')}" if strays else ""))
        samples = []
        lost = []
        for b in RTT_BYTES:
            ms, strays = rtt_one(port, b, timeout)
            if ms is None:
                lost.append(f"{b:02X}")
            else:
                samples.append(ms)
            if strays:
                notes.append(f"rtt {b:02X} strays {strays[:16].hex(' ')}")
        rate, n, got, wrong, span, why = measure_rate(port, baud, secs)
        verdicts = []
        if lost:
            verdicts.append(f"FAIL {len(lost)} rtt lost ({' '.join(lost)})")
        if wrong:
            verdicts.append(f"FAIL {wrong} bytes lost/wrong in the run")
            notes.append(f"run: {why}")
        if rate is None:
            verdicts.append("FAIL no rate (fewer than 2 reads)")
            err = 0.0
        else:
            err = (rate - baud) * 100.0 / baud
            if abs(err) > tol:
                verdicts.append(f"FAIL rate {err:+.1f}% off (over {tol:g}%)")
        all_ok &= not verdicts
        if samples:
            lo, hi = min(samples), max(samples)
            avg = sum(samples) / len(samples)
            rtt = (f"rtt min={lo:.2f} avg={avg:.2f} max={hi:.2f} ms "
                   f"= {avg / cms:.2f} char")
        else:
            rtt = "rtt none"
        rtxt = f"rate={rate:.1f} ({err:+.2f}%)" if rate is not None else "rate=n/a"
        print(f"  {baud:>7} baud: char={cms:.3g} ms set={port.speed()} | {rtt} | "
              f"{rtxt} n={n} got={got} span={span:.0f} ms | "
              + ("; ".join(verdicts) if verdicts else "ok"))
        for note in notes:
            print(f"           {note}")
    port.set_outputs(0, 0)
    return all_ok


def decode_parmrk(data):
    """Split a PARMRK stream (IGNBRK/BRKINT/INPCK off) into breaks, plain NUL
    bytes and other data bytes. 0xFF 0xFF is one literal 0xFF."""
    breaks = nuls = 0
    other = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == 0xFF and data[i + 1:i + 2] == b"\xff":
            other.append(0xFF)
            i += 2
        elif b == 0xFF and data[i + 1:i + 3] == b"\x00\x00":
            breaks += 1
            i += 3
        else:
            if b == 0:
                nuls += 1
            else:
                other.append(b)
            i += 1
    return breaks, nuls, bytes(other)


def read_timed(port, t0, deadline, phase):
    """Read until `deadline`, returning (ms since t0, phase, bytes) per read."""
    log = []
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return log
        if port.wait_readable(min(left, 0.005)):
            chunk = port.read()
            if chunk:
                log.append(((time.monotonic() - t0) * 1000.0, phase, chunk))


def check_break(port, bauds, holds, window):
    print("\n== Break (TX break -> RX, " + ("polled like 86Box's ClearCommError loop" if WIN
                                           else "read with PARMRK") + ") ==")
    all_ok = True
    for baud in bauds:
        port.configure(baud, parmrk=True)
        port.set_outputs(1, 1)
        for hold_ms in holds:
            for follow in (None, 0x41):
                t_off, log, case, why, note = port.break_trial(hold_ms / 1000.0, window, follow)
                all_ok &= case == "A"
                label = f"break {hold_ms} ms" + (f", then {follow:#04x}" if follow is not None else "")
                print(f"  {baud:>7} baud [{case}] {label}{note}: {why}")
                print(f"           break released at +{t_off:.1f} ms")
                if not log and WIN:
                    print("           (no errors, no data)")
                for ms, phase, text in log:
                    print(f"           +{ms:7.1f} ms  {phase:5}  {text}")
    port.set_outputs(0, 0)
    port.configure(bauds[0] if bauds else 9600)
    return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("device", nargs="?", default=default_device(),
                    help="serial device (default %(default)s)")
    ap.add_argument("--baud", default=SERTEST_BAUDS,
                    help="comma-separated rates for the data and timing tests (default %(default)s)")
    ap.add_argument("--data-secs", type=float, default=1.0,
                    help="line time per rate for the data test, at least 32 bytes (default %(default)s)")
    ap.add_argument("--rate-secs", type=float, default=2.0,
                    help="line time per rate for the rate measurement (default %(default)s)")
    ap.add_argument("--rate-tol", type=float, default=2.0,
                    help="percent the measured rate may be off nominal (default %(default)s)")
    ap.add_argument("--skip-timing", action="store_true")
    ap.add_argument("--break-baud", default="1200,9600,115200",
                    help="comma-separated rates for the break test (default %(default)s)")
    ap.add_argument("--expect", default=DEFAULT_EXPECT,
                    help="wiring to require, output-input pairs (default %(default)s)")
    ap.add_argument("--no-ri", action="store_true", help="don't require RI (adapters/plugs that lack it)")
    ap.add_argument("--settle", type=float, default=0.05, help="seconds to wait after changing a line")
    ap.add_argument("--skip-lines", action="store_true")
    ap.add_argument("--skip-data", action="store_true")
    ap.add_argument("--break-hold", default="10,100",
                    help="comma-separated break hold times in ms for the break test (default %(default)s)")
    ap.add_argument("--break-window", type=float, default=0.3,
                    help="seconds to keep reading after a break is released (default %(default)s)")
    ap.add_argument("--skip-break", action="store_true")
    ap.add_argument("--skip-driver", action="store_true", help="don't print the driver info section")
    a = ap.parse_args()

    def rates(opt, text):
        try:
            out = [int(b) for b in text.split(",") if b]
        except ValueError:
            ap.error(f"{opt} must be a comma-separated list of integers")
        for b in out:
            if b not in BAUD:
                ap.error(f"unsupported baud {b} (have {sorted(BAUD)})")
        return out
    bauds = rates("--baud", a.baud)
    break_bauds = rates("--break-baud", a.break_baud)
    try:
        holds = [int(h) for h in a.break_hold.split(",") if h]
    except ValueError:
        ap.error("--break-hold must be a comma-separated list of integers")

    try:
        port = open_port(a.device)
    except PortError as e:
        print(e, file=sys.stderr)
        return 2

    print(f"Device: {a.device}\n")
    if not a.skip_driver:
        show_driver(a.device, port)       # before configure(): Windows shows the driver's own DCB
        print()
    ok = True
    try:
        try:
            port.configure(bauds[0] if bauds else 9600)
            port.set_outputs(0, 0)
            port.lines()                # fail early if this isn't a modem-capable port
        except (OSError, PortError) as e:
            print(f"{a.device} does not support modem control ioctls: {e}", file=sys.stderr)
            return 2
        if not a.skip_lines:
            ok &= check_lines(port, a.settle, [p for p in a.expect.split(",") if p], not a.no_ri)
        if not a.skip_data:
            ok &= check_data(port, bauds, a.data_secs)
        if not a.skip_timing:
            ok &= check_timing(port, bauds, a.rate_secs, a.rate_tol)
        if not a.skip_break:
            ok &= check_break(port, break_bauds, holds, a.break_window)
    except PortError as e:
        print(f"\nport error: {e}", file=sys.stderr)
        return 2
    finally:
        try:
            port.set_outputs(0, 0)
        except (OSError, PortError):
            pass
        port.close()

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
