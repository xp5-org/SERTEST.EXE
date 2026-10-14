/* FIFOSW: does a 16550 to 16450 type switch turn the FIFO off?

   Amstrad MegaPC only. Bit 0 of WD76C30 index 5 picks the type, 0x01 is
   16550 and 0x00 is 16450. Index is at LPT base + 3, data at LPT base + 7.

   Bug fixed in ab_A: serial_set_type left the FIFO on after the switch.
   A 16450 should read IIR bits 7:6 as 00, hold one character and flag OE
   when a second arrives.

   Runs on COM1 in MCR loopback. Prints each check and a last line
   "FIFOSW DONE RESULT=PASS" or "FIFOSW DONE RESULT=FAIL <n>", also written
   to FIFOSW.TXT. The DONE line is upper-cased at run time.

   Build: wcl -0 -ms -w4 fifosw.c */

#include <stdio.h>
#include <stdarg.h>
#include <conio.h>
#include <dos.h>
#include <i86.h>

#define WD_IDX       3     /* WD76C30 index */
#define WD_DATA      7     /* WD76C30 data */
#define WD_REG_UART  0x05  /* UART type and clock */
#define WD_16550     0x01
#define WD_16450     0x00

#define NSEND        4     /* characters sent per buffering check */

static unsigned com;       /* COM1 base */
static unsigned lpt;       /* LPT1 base */
static FILE *logf;
static int fails;

static void say(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    if (logf) {
        va_start(ap, fmt);
        vfprintf(logf, fmt, ap);
        va_end(ap);
    }
}

/* lower-case literal, upper-cased at run time */
static void say_upper(const char *lower, int n)
{
    char buf[48];
    int i;

    for (i = 0; lower[i] && (i < (int) sizeof(buf) - 1); i++)
        buf[i] = (char) (((lower[i] >= 'a') && (lower[i] <= 'z')) ? lower[i] - 32 : lower[i]);
    buf[i] = 0;
    if (n)
        say("%s %d\n", buf, n);
    else
        say("%s\n", buf);
}

static void check(int ok, const char *what, const char *want, unsigned got)
{
    say("%s %-34s want %-8s got %02X\n", ok ? "PASS" : "FAIL", what, want, got);
    if (!ok)
        fails++;
}

/* BIOS tick count at 0040:006C */
static unsigned long ticks(void)
{
    return *(unsigned long far *) MK_FP(0x0040, 0x006C);
}

static void wait_ticks(unsigned n)
{
    unsigned long t0 = ticks();

    while ((ticks() - t0) < n)
        ;
}

static void wd_uart_type(unsigned char v)
{
    outp(lpt + WD_IDX, WD_REG_UART);
    outp(lpt + WD_DATA, v);
}

static unsigned char iir_fifo_bits(void)
{
    return (unsigned char) (inp(com + 2) & 0xC0);
}

/* drop anything received and clear LSR errors */
static void drain(void)
{
    int n = 0;

    while ((inp(com + 5) & 0x01) && (n++ < 64))
        (void) inp(com);
    (void) inp(com + 5);
}

/* LSR read that keeps error bits. Reading LSR clears them, so lsr_seen
   holds any OE seen. */
static unsigned char lsr_seen;

static unsigned char rd_lsr(void)
{
    unsigned char v = (unsigned char) inp(com + 5);

    lsr_seen |= v;
    return v;
}

/* Send NSEND characters in loopback without reading, then count how many
   are held and whether OE was flagged. */
static void buffering(unsigned *held, unsigned *oe)
{
    unsigned long t0;
    int i;

    drain();
    lsr_seen = 0;
    for (i = 0; i < NSEND; i++) {
        t0 = ticks();
        while (!(rd_lsr() & 0x20) && ((ticks() - t0) < 18))
            ;
        outp(com, 0x41 + i);
    }
    /* 2 ticks cover the last frame and receive delay */
    t0 = ticks();
    while (!(rd_lsr() & 0x40) && ((ticks() - t0) < 18))
        ;
    wait_ticks(2);

    *held = 0;
    while ((rd_lsr() & 0x01) && (*held < 64)) {
        (void) inp(com);
        (*held)++;
    }
    *oe = (lsr_seen & 0x02) ? 1 : 0;
}

static void check_buffering(const char *mode, int fifo)
{
    unsigned held, oe;
    char what[40];

    buffering(&held, &oe);
    sprintf(what, "%s: chars held of %d", mode, NSEND);
    check(fifo ? (held == NSEND) : (held == 1), what, fifo ? "04" : "01", held);
    sprintf(what, "%s: OE after %d chars", mode, NSEND);
    check(fifo ? !oe : oe, what, fifo ? "00" : "01", oe);
}

int main(void)
{
    unsigned far *bda = (unsigned far *) MK_FP(0x0040, 0x0000);
    unsigned char v;

    logf = fopen("FIFOSW.TXT", "w");
    com  = bda[0] ? bda[0] : 0x3F8;
    lpt  = bda[4];
    say("FIFOSW: MegaPC 16550 -> 16450 FIFO switch, COM1 %03X LPT1 %03X\n", com, lpt);
    if (!lpt) {
        say("FAIL no LPT1 in BDA 0040:0008: the WD76C30 regs sit at LPT base\n");
        say_upper("fifosw done result=fail", 1);
        if (logf)
            fclose(logf);
        return 1;
    }

    /* 9600 8N1, loopback, interrupts off */
    outp(com + 1, 0x00);
    outp(com + 3, 0x80);
    outp(com + 0, 12);
    outp(com + 1, 0);
    outp(com + 3, 0x03);
    outp(com + 4, 0x10);

    say("-- power-on type (86Box resets the WD76C30 UARTs to 16450)\n");
    outp(com + 2, 0x07);
    check(iir_fifo_bits() == 0x00, "16450: IIR 7:6 after FCR=07", "00", iir_fifo_bits());
    /* control: a never-switched 16450 */
    check_buffering("16450 power-on", 0);

    say("-- switch to 16550, FIFO on\n");
    wd_uart_type(WD_16550);
    outp(lpt + WD_IDX, WD_REG_UART);
    v = (unsigned char) inp(lpt + WD_DATA);
    check((v & 0x01) == WD_16550, "WD76C30 idx 5 reads back 16550", "01", v & 0x01);
    outp(com + 2, 0x07);
    check(iir_fifo_bits() == 0xC0, "16550: IIR 7:6 after FCR=07", "C0", iir_fifo_bits());
    check_buffering("16550 FIFO on", 1);

    say("-- switch to 16450 with the FIFO still on\n");
    wd_uart_type(WD_16450);
    check(iir_fifo_bits() == 0x00, "16450: IIR 7:6 after switch", "00", iir_fifo_bits());
    check_buffering("16450 after switch", 0);

    say("-- 16450: FCR=00 (a 16450 has no FCR)\n");
    outp(com + 2, 0x00);
    check(iir_fifo_bits() == 0x00, "16450: IIR 7:6 after FCR=00", "00", iir_fifo_bits());
    check_buffering("16450 after FCR=00", 0);

    /* leave it as the BIOS had it */
    outp(com + 4, 0x00);
    drain();

    if (fails)
        say_upper("fifosw done result=fail", fails);
    else
        say_upper("fifosw done result=pass", 0);
    if (logf)
        fclose(logf);
    return fails ? 1 : 0;
}
