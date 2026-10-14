import os, fcntl, struct, termios, tty, threading, select

VUART_IOC_MAGIC = ord('V')


def _iow(magic, nr, size):
    IOC_WRITE = 1
    return (IOC_WRITE << 30) | (size << 16) | (magic << 8) | nr


VUART_IOC_INJECT_FAULT = _iow(VUART_IOC_MAGIC, 1, 1)

a = os.open('/dev/vuart0', os.O_RDWR | os.O_NOCTTY)
b = os.open('/dev/vuart1', os.O_RDWR | os.O_NOCTTY)

tty.setraw(a)

attrs = termios.tcgetattr(b)
tty.setraw(b)
attrs = termios.tcgetattr(b)
iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
iflag |= termios.INPCK | termios.PARMRK
iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.ISTRIP | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
oflag &= ~termios.OPOST
lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
termios.tcsetattr(b, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

result = {}


def reader():
    r, _, _ = select.select([b], [], [], 5)
    if not r:
        result['status'] = 'TIMEOUT: nothing arrived within 5s'
        return
    data = os.read(b, 64)
    result['status'] = 'GOT: %r' % data
    result['is_pferr_escape'] = (data == b'\xff\x00\x5a')


t = threading.Thread(target=reader)
t.start()
import time
time.sleep(0.3)

print("injecting fault byte 0x5A via VUART_IOC_INJECT_FAULT on vuart0's PEER (vuart1)...")
fcntl.ioctl(a, VUART_IOC_INJECT_FAULT, struct.pack('B', 0x5A))

t.join(timeout=6)
print(result.get('status', 'THREAD DID NOT FINISH'))
if result.get('is_pferr_escape'):
    print("PASS: received the real termios PARMRK parity/framing escape (\\377\\0\\x5a) -- exactly what char_serial_unescape() expects")
else:
    print("FAIL: did not receive the expected \\377\\0\\x5a escape")

os.close(a)
os.close(b)
