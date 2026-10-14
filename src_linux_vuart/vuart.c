#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/termios.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#define VUART_N 2
#define VUART_NAME "vuart"

/* Fault injection for parity and framing errors. The peer sees the byte
 * escaped as the termios \377\0<byte> sequence on read. */
#define VUART_IOC_MAGIC 'V'
#define VUART_IOC_INJECT_FAULT _IOW(VUART_IOC_MAGIC, 1, unsigned char)
/* Arm or disarm the RTS->CTS and DTR->DSR|CAR|RNG loopback plug. */
#define VUART_IOC_ATTACH_LOOPBACK _IO(VUART_IOC_MAGIC, 2)
#define VUART_IOC_DETACH_LOOPBACK _IO(VUART_IOC_MAGIC, 3)

static struct tty_driver *vuart_driver;
static struct tty_port vuart_port[VUART_N];

/* Serializes every producer into a port's receive flip buffer. */
static spinlock_t vuart_rx_lock[VUART_N];

/* vuart_out[idx] is the MCR output of idx. When attached, vuart_tiocmget
 * translates it to CTS, DSR, CAR and RNG on every read. Detached, it
 * returns the peer's raw output. */
static DEFINE_SPINLOCK(vuart_mctrl_lock);
static unsigned int vuart_out[VUART_N];
/* Port that attached the plug, or -1. Cleared on that port's last close. */
static int vuart_loopback_owner = -1;

static unsigned int vuart_loopback_translate(unsigned int out_bits)
{
	unsigned int in_bits = 0;

	if (out_bits & TIOCM_RTS)
		in_bits |= TIOCM_CTS;
	if (out_bits & TIOCM_DTR)
		in_bits |= TIOCM_DSR | TIOCM_CAR | TIOCM_RNG;
	return in_bits;
}

static inline int peer_of(int idx)
{
	return idx ^ 1;
}

/* Queue bytes on idx's receive side. Returns -ENODEV if nobody has idx
 * open, else how many bytes fit. */
static ssize_t vuart_rx(int idx, const u8 *buf, size_t count, u8 flag)
{
	struct tty_port *port = &vuart_port[idx];
	struct tty_struct *tty;
	unsigned long flags;
	size_t n;

	tty = tty_port_tty_get(port);
	if (!tty)
		return -ENODEV;
	spin_lock_irqsave(&vuart_rx_lock[idx], flags);
	n = tty_insert_flip_string_fixed_flag(port, buf, flag, count);
	if (n)
		tty_flip_buffer_push(port);
	spin_unlock_irqrestore(&vuart_rx_lock[idx], flags);
	tty_kref_put(tty);
	return n;
}

static int vuart_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(&vuart_port[tty->index], tty, filp);
}

static void vuart_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(&vuart_port[tty->index], tty, filp);
}

/* A closed peer drops the bytes like an unplugged cable. A full peer
 * returns short so the writer waits for vuart_unthrottle. */
static ssize_t vuart_write(struct tty_struct *tty, const u8 *buf, size_t count)
{
	ssize_t n = vuart_rx(peer_of(tty->index), buf, count, TTY_NORMAL);

	return n < 0 ? count : n;
}

static unsigned int vuart_write_room(struct tty_struct *tty)
{
	struct tty_port *port = &vuart_port[peer_of(tty->index)];
	struct tty_struct *peer_tty = tty_port_tty_get(port);
	unsigned int room = 4096;

	if (peer_tty) {
		room = tty_buffer_space_avail(port);
		tty_kref_put(peer_tty);
	}
	return room;
}

/* The reader drained its buffer, so wake a writer blocked on a full peer. */
static void vuart_unthrottle(struct tty_struct *tty)
{
	tty_port_tty_wakeup(&vuart_port[peer_of(tty->index)]);
}

/* break_ctl forwards a TTY_BREAK flagged byte to the peer receive flip
 * buffer, so a break reads as a single LSR event. */
static int vuart_break_ctl(struct tty_struct *tty, int state)
{
	static const u8 zero;

	if (state != 0)
		vuart_rx(peer_of(tty->index), &zero, 1, TTY_BREAK);
	return 0;
}

static int vuart_ioctl(struct tty_struct *tty, unsigned int cmd,
			unsigned long arg)
{
	int idx = tty->index;
	unsigned long flags;
	u8 byte;

	switch (cmd) {
	case VUART_IOC_INJECT_FAULT:
		if (get_user(byte, (unsigned char __user *) arg))
			return -EFAULT;
		vuart_rx(peer_of(idx), &byte, 1, TTY_FRAME);
		return 0;
	case VUART_IOC_ATTACH_LOOPBACK:
		spin_lock_irqsave(&vuart_mctrl_lock, flags);
		vuart_loopback_owner = idx;
		spin_unlock_irqrestore(&vuart_mctrl_lock, flags);
		return 0;
	case VUART_IOC_DETACH_LOOPBACK:
		spin_lock_irqsave(&vuart_mctrl_lock, flags);
		if (vuart_loopback_owner == idx)
			vuart_loopback_owner = -1;
		spin_unlock_irqrestore(&vuart_mctrl_lock, flags);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

/* attached: loopback plug behavior. detached: the peer's raw output. */
static int vuart_tiocmget(struct tty_struct *tty)
{
	int idx = tty->index;
	unsigned long flags;
	int bits;

	spin_lock_irqsave(&vuart_mctrl_lock, flags);
	if (vuart_loopback_owner >= 0)
		bits = vuart_loopback_translate(vuart_out[idx]);
	else
		bits = vuart_out[peer_of(idx)];
	spin_unlock_irqrestore(&vuart_mctrl_lock, flags);
	return bits;
}

static int vuart_tiocmset(struct tty_struct *tty, unsigned int set,
			   unsigned int clear)
{
	int idx = tty->index;
	unsigned long flags;

	spin_lock_irqsave(&vuart_mctrl_lock, flags);
	vuart_out[idx] = (vuart_out[idx] & ~clear) | set;
	spin_unlock_irqrestore(&vuart_mctrl_lock, flags);
	return 0;
}

static int vuart_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	return 0;
}

/* Last close or hangup: drop a plug this port attached, and wake a peer
 * writer that was waiting on our full buffer. */
static void vuart_port_shutdown(struct tty_port *port)
{
	int idx = port - vuart_port;
	unsigned long flags;

	spin_lock_irqsave(&vuart_mctrl_lock, flags);
	if (vuart_loopback_owner == idx)
		vuart_loopback_owner = -1;
	spin_unlock_irqrestore(&vuart_mctrl_lock, flags);
	tty_port_tty_wakeup(&vuart_port[peer_of(idx)]);
}

static const struct tty_port_operations vuart_port_ops = {
	.activate = vuart_port_activate,
	.shutdown = vuart_port_shutdown,
};

static const struct tty_operations vuart_ops = {
	.open = vuart_open,
	.close = vuart_close,
	.write = vuart_write,
	.write_room = vuart_write_room,
	.unthrottle = vuart_unthrottle,
	.break_ctl = vuart_break_ctl,
	.ioctl = vuart_ioctl,
	.tiocmget = vuart_tiocmget,
	.tiocmset = vuart_tiocmset,
};

/* parent device for the ttys, gives the /sys/class/tty/vuartN/device/driver link */
static int vuart_pdrv_probe(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver vuart_pdrv = {
	.probe = vuart_pdrv_probe,
	.driver = {
		.name = VUART_NAME,
	},
};

static struct platform_device *vuart_pdev;

static int __init vuart_init(void)
{
	int i, ret;

	ret = platform_driver_register(&vuart_pdrv);
	if (ret)
		return ret;
	vuart_pdev = platform_device_register_simple(VUART_NAME, PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(vuart_pdev)) {
		platform_driver_unregister(&vuart_pdrv);
		return PTR_ERR(vuart_pdev);
	}

	vuart_driver = tty_alloc_driver(VUART_N, TTY_DRIVER_REAL_RAW |
					 TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(vuart_driver)) {
		ret = PTR_ERR(vuart_driver);
		goto err_pdev;
	}

	vuart_driver->driver_name = "vuart";
	vuart_driver->name = VUART_NAME;
	vuart_driver->type = TTY_DRIVER_TYPE_SERIAL;
	vuart_driver->subtype = SERIAL_TYPE_NORMAL;
	vuart_driver->init_termios = tty_std_termios;
	vuart_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;

	tty_set_operations(vuart_driver, &vuart_ops);

	for (i = 0; i < VUART_N; i++) {
		spin_lock_init(&vuart_rx_lock[i]);
		tty_port_init(&vuart_port[i]);
		vuart_port[i].ops = &vuart_port_ops;
		tty_port_link_device(&vuart_port[i], vuart_driver, i);
	}

	ret = tty_register_driver(vuart_driver);
	if (ret) {
		for (i = 0; i < VUART_N; i++)
			tty_port_destroy(&vuart_port[i]);
		tty_driver_kref_put(vuart_driver);
		goto err_pdev;
	}

	for (i = 0; i < VUART_N; i++)
		tty_register_device(vuart_driver, i, &vuart_pdev->dev);

	pr_info("vuart: /dev/vuart0 <-> /dev/vuart1 null-modem pair registered\n");
	return 0;

err_pdev:
	platform_device_unregister(vuart_pdev);
	platform_driver_unregister(&vuart_pdrv);
	return ret;
}

static void __exit vuart_exit(void)
{
	int i;

	for (i = 0; i < VUART_N; i++)
		tty_unregister_device(vuart_driver, i);
	tty_unregister_driver(vuart_driver);
	tty_driver_kref_put(vuart_driver);
	for (i = 0; i < VUART_N; i++)
		tty_port_destroy(&vuart_port[i]);
	platform_device_unregister(vuart_pdev);
	platform_driver_unregister(&vuart_pdrv);
}

module_init(vuart_init);
module_exit(vuart_exit);
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("null-modem tty pair with BREAK & RTS/DTR passthrough");
