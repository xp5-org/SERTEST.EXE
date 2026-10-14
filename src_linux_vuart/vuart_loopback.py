#!/usr/bin/env python3
"""testing tool : loopback plug for /dev/vuart0, the peer of 86Box COM1 on
/dev/vuart1.

writes to vuart1 : echoed back as RX, and RTS and DTR
reflect back as CTS, DSR, DCD and RI 

used in test inf on the vuart0 end. 

Modem lines are translated in the kernel by vuart_tiocmget. 
The loopback translation only runs while:
- VUART_IOC_ATTACH_LOOPBACK on open and VUART_IOC_DETACH_LOOPBACK on exit

Usage:
  vuart_loopback.py [--device /dev/vuart0] [--ready vuart_loopback.ready]

echoes forever, background it with nohup
"""

import argparse
import fcntl
import os
import select
import struct
import sys
import termios

# _IO(VUART_IOC_MAGIC='V', nr) from vuart.c, so (type << 8) | nr.
VUART_IOC_ATTACH_LOOPBACK = (ord("V") << 8) | 2
VUART_IOC_DETACH_LOOPBACK = (ord("V") << 8) | 3
# _IOW(VUART_IOC_MAGIC='V', 1, unsigned char), same as pferr_test.py.
VUART_IOC_INJECT_FAULT = (1 << 30) | (1 << 16) | (ord("V") << 8) | 1


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # Not cfmakeraw. char_serial.c sets INPCK|PARMRK on vuart1, so this end
    # needs the same flags. Without them a TTY_BREAK byte from break_ctl
    # arrives as a bare 0x00 and 86Box never sees a Break Interrupt.
    # The unescaper below turns the escapes back into real faults.
    attrs[0] = termios.INPCK | termios.PARMRK  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0  # lflag
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def attach_loopback(fd):
    fcntl.ioctl(fd, VUART_IOC_ATTACH_LOOPBACK)


def detach_loopback(fd):
    fcntl.ioctl(fd, VUART_IOC_DETACH_LOOPBACK)


def inject_fault(fd, byte):
    fcntl.ioctl(fd, VUART_IOC_INJECT_FAULT, struct.pack("B", byte))


class Unescaper:
    """Mirrors char_serial_unescape() in char_serial.c. With INPCK|PARMRK a
    break arrives as \\377\\0\\0, a parity/framing error as \\377\\0<byte> and a
    literal 0xFF as \\377\\377. Feed it one byte at a time. It yields
    ("data", byte) to echo, or ("fault", byte) to re-inject with
    VUART_IOC_INJECT_FAULT, where 0x00 is a break."""

    def __init__(self):
        self.state = 0

    def feed(self, b):
        if self.state == 1:
            self.state = 0
            if b == 0xFF:
                return ("data", 0xFF)
            if b == 0x00:
                self.state = 2
                return None
            # Not a documented termios sequence, keep the byte.
            return ("data", b)
        elif self.state == 2:
            self.state = 0
            return ("fault", b)
        else:
            if b == 0xFF:
                self.state = 1
                return None
            return ("data", b)


def write_bytes(fd, data):
    written = 0
    while written < len(data):
        try:
            written += os.write(fd, data[written:])
        except BlockingIOError:
            select.select([], [fd], [], 1)


def loop(fd, path):
    print(f"looping {path} back onto itself", flush=True)
    total = 0
    faults = 0
    unescaper = Unescaper()
    while True:
        readable, _, exceptional = select.select([fd], [], [fd], 0.02)
        if exceptional:
            raise IOError(f"exceptional condition on {path}")
        if fd in readable:
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b""
            if data == b"" and readable:
                # b"" from read() is a closed peer, unlike BlockingIOError.
                try:
                    probe = os.read(fd, 0)
                except OSError:
                    probe = None
                if probe == b"":
                    raise IOError(f"EOF on {path}")
            if data:
                # Echo before logging, since a flushed print to the NFS log adds
                # milliseconds per byte and fails external timing. Bytes go
                # through the unescaper. Break and parity/framing escapes are
                # re-injected as a real fault, not echoed as data.
                plain_run = bytearray()
                for b in data:
                    result = unescaper.feed(b)
                    if result is None:
                        continue
                    kind, val = result
                    if kind == "data":
                        plain_run.append(val)
                    else:
                        if plain_run:
                            write_bytes(fd, bytes(plain_run))
                            total += len(plain_run)
                            plain_run.clear()
                        inject_fault(fd, val)
                        faults += 1
                if plain_run:
                    write_bytes(fd, bytes(plain_run))
                    total += len(plain_run)
                print(f"{path}: echoed {len(data)} bytes {data!r} "
                      f"(total {total}, faults {faults})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/vuart0",
                    help="the vuart.ko node NOT held open by 86Box (default /dev/vuart0, peer of serial_passthrough's /dev/vuart1)")
    ap.add_argument("--ready", default=None,
                    help="touched once the device is open, the loopback plug is attached, and looping")
    ap.add_argument("--no-modem-lines", action="store_true",
                    help="byte loopback only -- don't attach the RTS/DTR->CTS/DSR/DCD/RI loopback-plug translation (e.g. against an older vuart.ko build without VUART_IOC_ATTACH_LOOPBACK)")
    args = ap.parse_args()

    if not os.path.exists(args.device):
        print(f"MISSING: {args.device} -- the vuart kernel module is not loaded "
              f"on this host. Rebuild/insmod vuart.ko (source kept alongside this "
              f"script) before rerunning.", file=sys.stderr)
        return 1

    try:
        fd = open_raw(args.device)
    except OSError as e:
        print(f"could not open {args.device}: {e}", file=sys.stderr)
        return 1

    attached = False
    if not args.no_modem_lines:
        try:
            attach_loopback(fd)
            attached = True
        except OSError as e:
            # Older vuart.ko without VUART_IOC_ATTACH_LOOPBACK. Byte loopback
            # still works, so fall back instead of crashing.
            print(f"{args.device}: attach-loopback ioctl failed ({e}) -- this "
                  f"vuart.ko build has no loopback-plug ioctl; continuing "
                  f"byte-loopback only, no RTS/DTR->CTS/DSR/DCD/RI translation",
                  file=sys.stderr)

    if args.ready:
        with open(args.ready, "w") as f:
            f.write(f"{args.device} (loopback plug: {'attached' if attached else 'detached'})\n")

    try:
        loop(fd, args.device)
    finally:
        if attached:
            try:
                detach_loopback(fd)
            except OSError:
                pass
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
