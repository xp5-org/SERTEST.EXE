/*
 * sertest_core.c -- the UART checks of SERTEST (page 1 and what it shares),
 * with no screen, keyboard, file or operating-system code in it.
 *
 * It is included, not compiled on its own: the program that includes it
 * (SERTEST.EXE's sertest.c, or a port such as the Linux puppet's) first
 * supplies, in the same translation unit,
 *   - sertest_hw.h and its hw_in / hw_out / hw_timer_read / ... names,
 *   - the shared state and constants (port, irq, cur_bi, cur_lcr, baud_tab, div_tab,
 *     pat_tab, the REG_, LSR_, MCR_ and MSR_ names, TICKS_*, chk_st, chk[], has_fifo, ...),
 *   - the timing helpers (t_start, t_now, t_sleep, hw_timer_read based), and
 *   - the reporting hooks the checks call:
 *       note(fmt, ...)          one failure detail line
 *       set_chk(panel, check, state)   a check's PEND/RUN/PASS/FAIL/SKIP/NC/NA row
 *       progress(done, total)   long checks report how far they are
 *       key_poll() / stop_req   let the user stop at an item boundary (a port with no
 *                               keyboard defines key_poll() as nothing and stop_req as 0)
 * The checks never read a key, draw, open a file or call the OS themselves.
 */
/* ----------------------------------------------------------------- UART */

static void uart_init(unsigned char mcr)
{
    unsigned int div = div_tab[cur_bi];
    hw_out(port + REG_IER, 0x00);
    hw_out(port + REG_LCR, 0x80);
    hw_out(port + REG_DLL, div & 0xFF);
    hw_out(port + REG_DLH, div >> 8);
    hw_out(port + REG_LCR, cur_lcr);
    hw_out(port + REG_FCR, 0x00);     /* FIFO off: strict per-byte behavior */
    hw_out(port + REG_MCR, mcr);
    (void) hw_in(port + REG_LSR);
    (void) hw_in(port + REG_MSR);
    (void) hw_in(port + REG_IIR);
}

static void drain_rx(void)
{
    int n = 0;
    while ((hw_in(port + REG_LSR) & LSR_DR) && n++ < 64)
        (void) hw_in(port + REG_DATA);
}

static void detect_uart(void)
{
    unsigned char iir;
    hw_out(port + REG_FCR, 0xC7);
    iir = (unsigned char) (hw_in(port + REG_IIR) & 0xC0);
    hw_out(port + REG_FCR, 0x00);
    has_fifo = (iir == 0xC0);
    has_scr = 1;
    if (iir == 0xC0)
        uart_name = "16550A";
    else if (iir == 0x80)
        uart_name = "16550";
    else {
        hw_out(port + REG_SCR, 0x5A);
        has_scr = (hw_in(port + REG_SCR) == 0x5A);
        uart_name = has_scr ? "16450" : "8250";
    }
}

/* ---- CPU class: detect_cpu() sets cpu_name; the code is the machine layer's. */
#define HW_IMPL_CPU
#include "sertest_hw.h"

/* ---- Poll speed. Back-to-back checks poll each byte and must go round
 * faster than the line delivers characters. Time the loop idle and call a
 * rate pollable when one 8N1 character lasts 3 idle passes. */
static unsigned long bit64(int i);

static void cal_poll(void)
{
    unsigned long t, n = 0;
    unsigned char l;
    int i;

    t_start();
    do {
        l = (unsigned char) hw_in(port + REG_LSR);
        t = t_now();
        if (l & LSR_DR)
            (void) hw_in(port + REG_DATA);
        n++;
    } while (t < 5UL * TICKS_10MS);
    poll16 = t * 16UL / n;
    for (i = NBAUD - 1; i > 0; i--)
        if (10UL * 16UL * bit64(i) / 64UL >= 3UL * poll16)
            break;
    cpu_top = i;
}

static unsigned char last_lsr;

static int top_bi(void) { return baud0 + nbaud - 1; }

static unsigned long bit64(int i);
static int wait_idle(void);

/* One character time at rate i and LCR lcr in PIT ticks, counted in
 * half bits. */
static unsigned long frame_ticks(int i, unsigned char lcr)
{
    unsigned long half = 2 + 2 * (5 + (lcr & 3)) + ((lcr & 0x08) ? 2 : 0)
                         + ((lcr & 0x04) ? ((lcr & 3) == 0 ? 3 : 4) : 2);
    return half * bit64(i) / 128;
}

/* One byte out, time until LSR.DR, returns what came back. last_lsr keeps
 * the LSR that reported DR. */
static int xfer_byte(unsigned char b, unsigned long *ticks, unsigned char *got)
{
    unsigned char l;
    t_start();
    while (!(hw_in(port + REG_LSR) & LSR_THRE))
        if (t_now() >= TMO_TICKS)
            return 0;
    drain_rx();
    t_start();
    hw_out(port + REG_DATA, b);
    while (!((l = (unsigned char) hw_in(port + REG_LSR)) & LSR_DR))
        if (t_now() >= TMO_TICKS)
            return 0;
    *ticks = t_now();
    last_lsr = l;
    *got = (unsigned char) hw_in(port + REG_DATA);
    return 1;
}

/* Full 00-FF at the top active rate, 8N1. */
static int data_test(unsigned char mcr)
{
    unsigned long t;
    unsigned char got;
    int b, ok = 1, tmo = 0;

    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    for (b = 0; b < 256 && tmo < 3; b++) {
        progress(b, 256);
        if (!xfer_byte((unsigned char) b, &t, &got)) {
            ok = 0;
            tmo++;
        } else if (got != (unsigned char) b)
            ok = 0;
    }
    progress(256, 256);
    return ok;
}

/* LCR as text: 8N1, 7E2, 5N1.5. */
static void fmt_name(char *buf, unsigned char lcr)
{
    static const char par[8] = { 'N', 'O', 'N', 'E', 'N', 'M', 'N', 'S' };
    int bits = 5 + (lcr & 3);
    sprintf(buf, "%d%c%s", bits, par[(lcr >> 3) & 7],
            (lcr & 0x04) ? (bits == 5 ? "1.5" : "2") : "1");
}

/* 5-8 data bits x N/O/E/M/S parity x 1/2 stop bits, 40 LCR values, at
 * rate bi. Each pattern must come back masked to the word length with no
 * error bits. Below 1200 baud the patterns drop to 00 FF 55 AA. */
static int format_test(unsigned char mcr, int bi)
{
    static const unsigned char par[5] = { 0x00, 0x08, 0x18, 0x28, 0x38 };
    unsigned long t;
    unsigned char got, mask;
    char fn[8];
    int wl, pi, sb, p, np, ok = 1, tmo = 0;

    cur_bi = bi;
    np = baud_tab[cur_bi] >= 1200 ? 8 : 4;
    for (wl = 0; wl < 4 && tmo < 3; wl++) {
        mask = (unsigned char) ((1 << (5 + wl)) - 1);
        for (pi = 0; pi < 5 && tmo < 3; pi++)
            for (sb = 0; sb < 2 && tmo < 3; sb++) {
                progress(wl * 10 + pi * 2 + sb, 40);
                cur_lcr = (unsigned char) (wl | par[pi] | (sb << 2));
                fmt_name(fn, cur_lcr);
                uart_init(mcr);
                for (p = 0; p < np; p++) {
                    if (!xfer_byte(pat_tab[p], &t, &got)) {
                        ok = 0;
                        tmo++;
                        note("fmt %lu %s: %02X timed out", baud_tab[bi], fn, pat_tab[p]);
                        break;
                    }
                    if ((got & mask) != (pat_tab[p] & mask) || (last_lsr & 0x0E)) {
                        ok = 0;
                        note("fmt %lu %s: sent %02X got %02X lsr %02X", baud_tab[bi], fn, pat_tab[p] & mask, got, last_lsr);
                    }
                }
            }
    }
    if (tmo >= 3)
        note("fmt %lu: 3 timeouts, rest skipped", baud_tab[bi]);
    progress(40, 40);
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    return ok;
}

/* ---- FIFO burst: pseudo-random bytes back to back with 16550 FIFOs on.
 * Makes 86Box keep up at line rate. A unique sequence shows a dropped,
 * doubled or reordered byte. Seeded per rate, format and panel. */

#define BURST_MAX 256
static unsigned char burst_buf[BURST_MAX];
static unsigned int  rnd_state;
static int burst_done, burst_total;     /* bytes back so far / expected, this rate */

static unsigned char rnd8(void)
{
    rnd_state = rnd_state * 25173u + 13849u;
    return (unsigned char) (rnd_state >> 8);
}

static int burst_len(unsigned long b)
{
    return b >= 9600 ? 256 : (b >= 1200 ? 128 : 32);
}

static int burst_one(unsigned char mcr, unsigned char lcr, int n, unsigned int seed)
{
    unsigned char mask = (unsigned char) ((1 << (5 + (lcr & 3))) - 1), lsr, c, err = 0, badc = 0;
    unsigned long last = 0, char_ticks = 12UL * PIT_HZ / baud_tab[cur_bi];
    int i, k, sent = 0, recv = 0, extra = 0, bad = -1;
    char fn[8];

    cur_lcr = lcr;
    uart_init(mcr);
    hw_out(port + REG_FCR, 0x07);         /* FIFOs on, both cleared, RX trigger 1 */
    drain_rx();
    rnd_state = seed;
    for (i = 0; i < n; i++)
        burst_buf[i] = (unsigned char) (rnd8() & mask);

    t_start();
    while (recv < n) {
        lsr = (unsigned char) hw_in(port + REG_LSR);
        err |= (unsigned char) (lsr & 0x8E);   /* OE PE FE, RX FIFO error */
        if (lsr & LSR_DR) {
            c = (unsigned char) hw_in(port + REG_DATA);
            if (bad < 0 && c != burst_buf[recv]) {
                bad = recv;
                badc = c;
            }
            recv++;
            progress(burst_done + recv, burst_total);
            last = t_now();
            continue;
        }
        if ((lsr & LSR_THRE) && sent < n)       /* TX FIFO empty: refill 16 */
            for (k = 0; k < 16 && sent < n; k++)
                hw_out(port + REG_DATA, burst_buf[sent++]);
        if (t_now() - last > TMO_TICKS)          /* 1s with no byte back */
            break;
    }
    /* anything still arriving after the burst is a duplicate */
    t_sleep(4 * char_ticks);
    while ((hw_in(port + REG_LSR) & LSR_DR) && extra < 64) {
        (void) hw_in(port + REG_DATA);
        extra++;
    }
    hw_out(port + REG_FCR, 0x00);

    if (recv == n && bad < 0 && !err && !extra)
        return 1;
    fmt_name(fn, lcr);
    if (recv < n)
        note("burst %lu %s: %d/%d back", baud_tab[cur_bi], fn, recv, n);
    if (bad >= 0)
        note("burst %lu %s: #%d sent %02X got %02X", baud_tab[cur_bi], fn, bad, burst_buf[bad], badc);
    if (err)
        note("burst %lu %s: lsr errors %02X", baud_tab[cur_bi], fn, err);
    if (extra)
        note("burst %lu %s: %d extra bytes", baud_tab[cur_bi], fn, extra);
    return 0;
}

/* The burst rate: the top active rate, or the fastest the CPU can poll if
 * lower. -1 when that is below the active slice. */
static int burst_bi(void)
{
    if (top_bi() <= cpu_top)
        return top_bi();
    return cpu_top >= baud0 ? cpu_top : -1;
}

/* 8N1, 7E1, 6O2, 5N1.5 at burst_bi. Needs a 16550A since the TX refill
 * writes 16 bytes at once. */
static int burst_test(unsigned char mcr, unsigned int panel_seed)
{
    static const unsigned char lcrs[4] = { 0x03, 0x1A, 0x0D, 0x04 };
    int f, ok = 1, n;

    cur_bi = burst_bi();
    n = burst_len(baud_tab[cur_bi]);
    burst_done = 0;
    burst_total = 4 * n;
    progress(0, burst_total);
    for (f = 0; f < 4; f++) {
        if (!burst_one(mcr, lcrs[f], n, (unsigned int) ((cur_bi << 8) ^ (f * 97) ^ panel_seed)))
            ok = 0;
        burst_done += n;
    }
    progress(burst_total, burst_total);
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    return ok;
}

static int wait_msr(unsigned char mask, unsigned char want, unsigned long *ticks)
{
    unsigned long t;
    for (;;) {
        t = t_now();
        if ((hw_in(port + REG_MSR) & mask) == want) {
            *ticks = t;
            return 1;
        }
        if (t >= TMO_TICKS)
            return 0;
    }
}

/* Toggle one MCR output nsamp times and time the MSR input following it.
 * base holds the other MCR bits. *worst, if not NULL, is raised to the
 * slowest edge. */
static int line_test(unsigned char base, unsigned char obit, unsigned char ibit,
                     int nsamp, unsigned long *worst)
{
    unsigned long t;
    int i, rising, ok = 1;

    for (i = 0; i < nsamp; i++) {
        rising = !(i & 1);
        hw_out(port + REG_MCR, rising ? base : (base | obit));
        t_start();
        if (!wait_msr(ibit, rising ? 0 : ibit, &t)) {
            ok = 0;                          /* never settled to the start state */
            t = TMO_TICKS;
        } else {
            t_start();
            hw_out(port + REG_MCR, rising ? (base | obit) : base);
            if (!wait_msr(ibit, rising ? ibit : 0, &t)) {
                ok = 0;
                t = TMO_TICKS;
            }
        }
        if (worst && t > *worst)
            *worst = t;
    }
    hw_out(port + REG_MCR, base);
    return ok;
}

/* One interrupt per check. The first records IIR and sets IER to 0, then
 * each handler clears what it raised. */
static HW_ISR(com_isr)
{
    unsigned char iir = (unsigned char) HW_ISR_IIR();
    if (isr_hits++ == 0) {
        isr_iir = iir;
        hw_out(port + REG_IER, 0x00);
    }
    isr_lsr |= (unsigned char) hw_in(port + REG_LSR);
    while (hw_in(port + REG_LSR) & LSR_DR)
        isr_data = hw_in(port + REG_DATA);
    (void) hw_in(port + REG_MSR);
    hw_out(PIC1_CMD, 0x20);
}

static int wait_thre(void)
{
    t_start();
    while (!(hw_in(port + REG_LSR) & LSR_THRE))
        if (t_now() >= TMO_TICKS)
            return 0;
    return 1;
}

/* Overrun, FIFO off: three bytes back to back, none read. A real 8250
 * keeps the newest in RBR and sets OE. Only THRE is polled, and only before
 * the last byte, so OE survives to the final read. */
static int overrun_test(unsigned char mcr)
{
    unsigned long ch;
    unsigned char l, d, l2;
    int ok;

    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    ch = frame_ticks(cur_bi, cur_lcr);
    if (!wait_idle())
        return 0;
    (void) hw_in(port + REG_LSR);
    hw_out(port + REG_DATA, 0x11);
    if (!wait_thre())
        return 0;
    hw_out(port + REG_DATA, 0x22);
    if (!wait_thre())
        return 0;
    hw_out(port + REG_DATA, 0x33);
    t_sleep(20 * ch + TICKS_100MS / 2);
    l = (unsigned char) hw_in(port + REG_LSR);
    d = (unsigned char) hw_in(port + REG_DATA);
    l2 = (unsigned char) hw_in(port + REG_LSR);
    ok = (l & (LSR_DR | LSR_OE)) == (LSR_DR | LSR_OE) && d == 0x33 && !(l2 & (LSR_DR | LSR_OE));
    if (!ok)
        note("overrun %lu: LSR %02X RBR %02X, then LSR %02X (want 03 33, then neither)",
             baud_tab[cur_bi], l, d, l2);
    drain_rx();
    return ok;
}

/* ---- Break: LCR bit 6 held BRK_HOLD_CH character times, then off. A real
 * UART gives one 0x00 with BI for the whole break.
 *   polled  every character read as it arrives: one 0x00 with BI, no OE.
 *   late    nothing read until BRK_SETTLE after the break, then one LSR
 *           read: DR, BI, no OE, RBR 0x00. A second character shows as OE.
 * A host adapter reports a break only after about 15 character times. */

#define BRK_HOLD_CH 40
#define BRK_SETTLE  (3UL * TICKS_100MS)

static int break_one(unsigned char mcr, int bi, unsigned char lcr, int late)
{
    unsigned long ch = frame_ticks(bi, lcr), hold = BRK_HOLD_CH * ch, settle = 20 * ch + BRK_SETTLE, t;
    unsigned char l, l2 = 0, d = 0, c, got[3], gl[3];
    int n = 0, i, ok, off = 0;
    char fn[8], buf[40];

    if (hold < TICKS_100MS)
        hold = TICKS_100MS;
    cur_bi = bi;
    cur_lcr = lcr;
    uart_init(mcr);
    fmt_name(fn, lcr);
    if (!wait_idle())
        return 0;
    t_sleep(4 * ch);
    drain_rx();
    (void) hw_in(port + REG_LSR);

    hw_out(port + REG_LCR, (unsigned char) (lcr | 0x40));
    if (late) {
        t_sleep(hold);
        hw_out(port + REG_LCR, lcr);
        t_sleep(settle);
        l = (unsigned char) hw_in(port + REG_LSR);
        if (l & LSR_DR)
            d = (unsigned char) hw_in(port + REG_DATA);
        l2 = (unsigned char) hw_in(port + REG_LSR);
        ok = (l & (LSR_DR | LSR_OE | LSR_BI)) == (LSR_DR | LSR_BI) && d == 0x00 && !(l2 & LSR_DR);
        if (!ok)
            note("brk %lu %s late: LSR %02X RBR %02X, then LSR %02X (want 11+ 00, no DR)",
                 baud_tab[bi], fn, l, d, l2);
    } else {
        t_start();
        for (;;) {
            t = t_now();
            if (!off && t >= hold) {
                hw_out(port + REG_LCR, lcr);
                off = 1;
                t_start();
                continue;
            }
            if (off && t >= settle)
                break;
            l = (unsigned char) hw_in(port + REG_LSR);
            if (l & LSR_DR) {
                c = (unsigned char) hw_in(port + REG_DATA);
                if (n < 3) {
                    got[n] = c;
                    gl[n] = l;
                }
                n++;
            }
        }
        ok = n == 1 && got[0] == 0x00 && (gl[0] & (LSR_OE | LSR_BI)) == LSR_BI;
        if (!ok) {
            buf[0] = 0;
            for (i = 0; i < n && i < 3; i++)
                sprintf(buf + strlen(buf), " %02X/%02X", got[i], gl[i]);
            note("brk %lu %s polled: %d chars%s (want 1: 00/BI)", baud_tab[bi], fn, n, buf);
        }
    }
    drain_rx();
    return ok;
}

static int break_test(unsigned char mcr, int late)
{
    int ok = break_one(mcr, top_bi(), 0x03, late);
    if (!break_one(mcr, BI_2400, 0x1A, late))
        ok = 0;
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    return ok;
}

/* -------------------------------------------------------------- panels */

#define RUN_CHK(p, i, expr) do { key_poll(); if (stop_req) break; set_chk(p, i, ST_RUN); set_chk(p, i, (expr) ? ST_PASS : ST_FAIL); } while (0)

static int scratch_test(void)
{
    static const unsigned char pat[4] = { 0x55, 0xAA, 0x00, 0xFF };
    int i, ok = 1;
    for (i = 0; i < 4; i++) {
        hw_out(port + REG_SCR, pat[i]);
        if ((unsigned char) hw_in(port + REG_SCR) != pat[i])
            ok = 0;
    }
    return ok;
}

/* THRE and TEMT both drop when THR is written, then return once the byte
 * shifts out. */
static int thre_test(unsigned char mcr)
{
    unsigned char l;
    int ok = 0;

    uart_init(mcr);
    t_start();
    while ((hw_in(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT) && t_now() < TMO_TICKS)
        ;
    hw_out(port + REG_DATA, 0x3C);
    l = (unsigned char) hw_in(port + REG_LSR);
    if (!(l & LSR_TEMT)) {
        t_start();
        while ((hw_in(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT) && t_now() < TMO_TICKS)
            ;
        ok = (hw_in(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) == (LSR_THRE | LSR_TEMT);
    }
    t_sleep(TICKS_10MS);
    drain_rx();
    return ok;
}

/* LSR & mask == want within TMO_TICKS. */
static int wait_lsr(unsigned char mask, unsigned char want)
{
    t_start();
    while ((hw_in(port + REG_LSR) & mask) != want && t_now() < TMO_TICKS)
        ;
    return (hw_in(port + REG_LSR) & mask) == want;
}

/* FCR bit 0 switches FIFO mode and clears both FIFOs.
 * A: FIFO off to on with a byte in THR. B: on to off with bytes queued.
 *    THRE and TEMT must settle.
 * C: a received byte waiting. The switch clears it so DR reads 0. */
static int fcrsw_test(unsigned char mcr)
{
    const unsigned char idle = LSR_THRE | LSR_TEMT;
    unsigned char l;
    int ok = 1, i, m;

    uart_init(mcr);
    (void) wait_lsr(idle, idle);
    hw_out(port + REG_DATA, 0x3C);
    hw_out(port + REG_FCR, 0x01);
    if (!wait_lsr(idle, idle)) {
        note("fcrsw off->on, byte in THR: THRE/TEMT stuck, LSR %02X", hw_in(port + REG_LSR));
        ok = 0;
    }
    t_sleep(TICKS_100MS);
    drain_rx();

    uart_init(mcr);
    (void) wait_lsr(idle, idle);
    hw_out(port + REG_FCR, 0x07);
    for (i = 0; i < 4; i++)
        hw_out(port + REG_DATA, (unsigned char) (0xA0 + i));
    hw_out(port + REG_FCR, 0x00);
    if (!wait_lsr(idle, idle)) {
        note("fcrsw on->off, bytes queued: THRE/TEMT stuck, LSR %02X", hw_in(port + REG_LSR));
        ok = 0;
    }
    t_sleep(TICKS_100MS);
    drain_rx();

    for (m = 0; m < 2; m++) {
        uart_init(mcr);
        (void) wait_lsr(idle, idle);
        if (m)
            hw_out(port + REG_FCR, 0x07);
        hw_out(port + REG_DATA, 0x5A);
        if (!wait_lsr(LSR_DR, LSR_DR)) {
            note("fcrsw %s: no DR before the switch", m ? "on->off" : "off->on");
            ok = 0;
            continue;
        }
        hw_out(port + REG_FCR, m ? 0x00 : 0x01);
        l = (unsigned char) hw_in(port + REG_LSR);
        if (l & (LSR_DR | 0x80)) {
            note("fcrsw %s, byte waiting: LSR %02X after the switch (want DR and bit 7 clear)",
                 m ? "on->off" : "off->on", l);
            ok = 0;
        }
        t_sleep(TICKS_100MS);
        drain_rx();
    }

    hw_out(port + REG_FCR, 0x00);
    return ok;
}

/* ---- WD76C30 UART type switch, Amstrad MegaPC. Index at LPT base + 3,
 * data at LPT base + 7. Register 5 bit 0 is 1 for 16550 and 0 for 16450.
 * Change it read-modify-write and keep the index clock bits. */
static void wd_set(unsigned char r5)
{
    hw_out(wd_lpt + 3, (wd_sel & 0xF8) | 0x05);
    hw_out(wd_lpt + 7, r5);
}

static void wd_restore(void)
{
    wd_set(wd_r5);
    hw_out(wd_lpt + 3, wd_sel);
}

/* Only on a UART that reads as 16450 with an LPT1. Setting register 5
 * bit 0 must give the UART a FIFO, IIR 7:6 = C0. FCR=00 before switching
 * back. */
static void detect_typesw(void)
{
    unsigned int lpt = *(hw_wptr) hw_bda(0x0008);
    unsigned char iir;

    wd_lpt = 0;
    if (has_fifo || !has_scr || !lpt)
        return;
    wd_sel = (unsigned char) hw_in(lpt + 3);
    hw_out(lpt + 3, (wd_sel & 0xF8) | 0x05);
    if ((unsigned char) hw_in(lpt + 3) != ((wd_sel & 0xF8) | 0x05)) {
        hw_out(lpt + 3, wd_sel);
        return;
    }
    wd_r5 = (unsigned char) hw_in(lpt + 7);
    wd_lpt = lpt;
    wd_set((unsigned char) (wd_r5 | 0x01));
    hw_out(port + REG_FCR, 0x07);
    iir = (unsigned char) (hw_in(port + REG_IIR) & 0xC0);
    hw_out(port + REG_FCR, 0x00);
    wd_restore();
    if (iir != 0xC0)
        wd_lpt = 0;
}

/* NSEND bytes back to back, none read until the last lands, then how many
 * RBR or the RX FIFO holds, the last one read, and OE. Same polling rule as
 * overrun_test. */
#define TYPESW_N 4

static void typesw_buf(int *held, int *oe, unsigned char *last)
{
    unsigned long ch = frame_ticks(cur_bi, cur_lcr);
    unsigned char l;
    int i;

    *held = *oe = 0;
    *last = 0;
    if (!wait_idle())
        return;
    (void) hw_in(port + REG_LSR);
    for (i = 0; i < TYPESW_N; i++) {
        if (i && !wait_thre())
            return;
        hw_out(port + REG_DATA, (unsigned char) (0x41 + i));
    }
    t_sleep(20 * ch + TICKS_100MS / 2);
    l = (unsigned char) hw_in(port + REG_LSR);
    while ((l & LSR_DR) && *held < 64) {
        if (l & LSR_OE)
            *oe = 1;
        *last = (unsigned char) hw_in(port + REG_DATA);
        (*held)++;
        l = (unsigned char) hw_in(port + REG_LSR);
    }
    if (l & LSR_OE)
        *oe = 1;
}

/* One phase: fifo=1 wants IIR C0, 4 held, no OE. fifo=0 wants IIR 00,
 * 1 held, the last byte sent, and OE. */
static int typesw_one(const char *what, int fifo)
{
    unsigned char iir = (unsigned char) (hw_in(port + REG_IIR) & 0xC0), last;
    int held, oe, ok;

    typesw_buf(&held, &oe, &last);
    ok = fifo ? (iir == 0xC0 && held == TYPESW_N && !oe)
              : (iir == 0x00 && held == 1 && oe && last == 0x40 + TYPESW_N);
    if (!ok)
        note("typesw %s: IIR %02X held %d OE %d RBR %02X (want %s)", what, iir, held, oe, last,
             fifo ? "C0 4 0 44" : "00 1 1 44");
    return ok;
}

static int typesw_test(unsigned char mcr)
{
    int ok = 1;

    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    wd_set((unsigned char) (wd_r5 | 0x01));
    hw_out(port + REG_FCR, 0x07);
    ok &= typesw_one("16550 FIFO on", 1);
    wd_set((unsigned char) (wd_r5 & ~0x01));
    ok &= typesw_one("->16450", 0);
    hw_out(port + REG_FCR, 0x00);
    ok &= typesw_one("16450 FCR=00", 0);
    wd_restore();
    uart_init(mcr);
    drain_rx();
    return ok;
}

/* Delta bits: raise every line together and OR in each delta seen, since
 * MSR reads clear them. DCTS DDSR DDCD must be seen, TERI not, and a re-read
 * must find them cleared. Then drop the RI driver alone and TERI must set.
 * INT drives RTS DTR OUT1 OUT2, EXT drives RTS and DTR. */
static int delta_test(int p)
{
    unsigned char base = p ? 0x00 : MCR_LOOP;
    unsigned char up = p ? (MCR_RTS | MCR_DTR) : (MCR_RTS | MCR_DTR | MCR_OUT1 | MCR_OUT2);
    unsigned char ri_drv = p ? MCR_DTR : MCR_OUT1;
    unsigned char lines = MSR_CTS | MSR_DSR | MSR_DCD;
    const unsigned char deltas = MSR_DCTS | MSR_DDSR | MSR_DDCD;
    unsigned char m, seen = 0;
    unsigned long t;
    int ri = !(p && no_ri), ok;

    if (ri)
        lines |= MSR_RI;
    uart_init(base);
    t_start();
    if (!wait_msr(lines, 0, &t)) {
        note("delta: lines never all dropped");
        return 0;
    }
    t_sleep(TICKS_10MS);
    (void) hw_in(port + REG_MSR);
    hw_out(port + REG_MCR, base | up);
    t_start();
    do {
        m = (unsigned char) hw_in(port + REG_MSR);
        seen |= m & 0x0F;
    } while ((m & lines) != lines && t_now() < TMO_TICKS);
    t_sleep(TICKS_1MS);
    seen |= (unsigned char) (hw_in(port + REG_MSR) & 0x0F);
    m = (unsigned char) hw_in(port + REG_MSR);
    ok = (seen & deltas) == deltas && !(seen & MSR_TERI) && !(m & 0x0F);
    if (!ok)
        note("delta rise: deltas %02X, then %02X (want %02X, then 00)", seen, m & 0x0F, deltas);
    if (ri) {
        seen = 0;
        hw_out(port + REG_MCR, base | (up & ~ri_drv));
        t_start();
        do {
            m = (unsigned char) hw_in(port + REG_MSR);
            seen |= m & 0x0F;
        } while ((m & MSR_RI) && t_now() < TMO_TICKS);
        t_sleep(TICKS_1MS);
        seen |= (unsigned char) (hw_in(port + REG_MSR) & 0x0F);
        if (!(seen & MSR_TERI)) {
            ok = 0;
            note("delta: RI fell, deltas %02X (no TERI 04)", seen);
        }
    }
    hw_out(port + REG_MCR, base);
    return ok;
}

/* ---- Interrupts: hook the vector, unmask the PIC line, enable one source
 * in IER, provoke it and check the ISR saw IIR report it. mcr must include
 * OUT2, which gates the IRQ line.
 *   RX       IER 1: one byte, IIR 4, the ISR reads A5
 *   THRE     IER 2: THR empty, IIR 2
 *   LSR      IER 4: overrun of three bytes, IIR 6, LSR OE
 *   MSR      IER 8: RTS toggled, DCTS, IIR 0
 *   TMO      IER 1, FIFO trigger 4: one byte below the trigger, IIR C after
 *            the timeout, the ISR reads A5, 16550A only */

enum { IRQ_RX, IRQ_THRE, IRQ_LSR, IRQ_MSR, IRQ_TMO };

static int irq_check(unsigned char mcr, int kind)
{
    static const unsigned char ier[5]  = { 0x01, 0x02, 0x04, 0x08, 0x01 };
    static const unsigned char want[5] = { 0x04, 0x02, 0x06, 0x00, 0x0C };
    static const char *what[5] = { "RX data", "THR empty", "line status", "modem status", "FIFO timeout" };
    unsigned char vec = (unsigned char) (irq + 8), bit = (unsigned char) (1 << irq), mask;
    int ok;

    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(mcr);
    t_sleep(TICKS_10MS);
    drain_rx();
    if (kind == IRQ_TMO)
        hw_out(port + REG_FCR, 0x47);     /* FIFOs on and cleared, RX trigger 4 */
    (void) hw_in(port + REG_LSR);
    (void) hw_in(port + REG_MSR);
    (void) hw_in(port + REG_IIR);
    isr_hits = 0;
    isr_data = -1;
    isr_iir = 0xFF;
    isr_lsr = 0;

    HW_IRQ_ARM(vec, bit, mask);
    hw_out(port + REG_IER, ier[kind]);

    switch (kind) {
    case IRQ_LSR:
        if (wait_thre()) {
            hw_out(port + REG_DATA, 0x11);
            if (wait_thre()) {
                hw_out(port + REG_DATA, 0x22);
                if (wait_thre())
                    hw_out(port + REG_DATA, 0x33);
            }
        }
        break;
    case IRQ_MSR:
        hw_out(port + REG_MCR, mcr ^ MCR_RTS);
        break;
    default:
        if (wait_thre())
            hw_out(port + REG_DATA, 0xA5);
        break;
    }
    t_start();
    while (isr_hits == 0 && t_now() < TMO_TICKS)
        HW_IRQ_POLL(bit);

    hw_out(port + REG_IER, 0x00);
    HW_IRQ_DISARM(vec, bit, mask);
    hw_out(port + REG_FCR, 0x00);
    hw_out(port + REG_MCR, mcr);

    ok = isr_hits > 0 && (isr_iir & 0x0F) == want[kind];
    if (kind == IRQ_RX || kind == IRQ_TMO)
        ok = ok && isr_data == 0xA5;
    if (kind == IRQ_LSR)
        ok = ok && (isr_lsr & LSR_OE);
    if (!ok)
        note("irq %s: %u hit(s), IIR %02X (want %X), data %d, LSR %02X",
             what[kind], isr_hits, isr_iir, want[kind], isr_data, isr_lsr);
    t_sleep(TICKS_10MS);
    drain_rx();
    return ok;
}

/* Every page 1 check on one panel, 0 INT or 1 EXT. */
static void run_panel(int p)
{
    unsigned char base = p ? 0x00 : MCR_LOOP;
    unsigned char irqm = p ? (MCR_OUT2 | MCR_RTS | MCR_DTR) : (MCR_LOOP | MCR_OUT2);
    int i;

    note_panel = p ? "EXT" : "INT";
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(base);

    if (p) {
        t_sleep(TICKS_100MS);           /* let a plug see the lines drop */
        key_poll();
        if (stop_req) {
            uart_init(0x00);
            return;
        }
        ext_noplug = 0;
        set_chk(1, C_PLUG, ST_RUN);
        if (!line_test(0x00, MCR_RTS, MSR_CTS, 2, NULL)) {
            ext_noplug = 1;
            note("no plug: RTS never came back on CTS");
            set_chk(1, C_PLUG, ST_FAIL);
            for (i = 0; i < NCHK; i++)
                if (chk_st[1][i] == ST_PEND)
                    set_chk(1, i, ST_SKIP);
            return;
        }
        set_chk(1, C_PLUG, ST_PASS);
    } else if (has_scr)
        RUN_CHK(0, C_SCRATCH, scratch_test());
    else
        set_chk(0, C_SCRATCH, ST_NA);       /* 8250: no scratch register */

    RUN_CHK(p, C_DATA, data_test(base));
    RUN_CHK(p, C_FMT, format_test(base, top_bi()));
    if (top_bi() == BI_2400)
        set_chk(p, C_FMT2400, ST_NA);   /* the row above already ran at 2400 */
    else
        RUN_CHK(p, C_FMT2400, format_test(base, BI_2400));
    if (has_fifo && burst_bi() >= 0)
        RUN_CHK(p, C_BURST, burst_test(base, p ? 0x5A5A : 0x1234));
    else
        set_chk(p, C_BURST, ST_NA);
    RUN_CHK(p, C_OVERRUN, overrun_test(base));
    if (p && no_brk) {
        set_chk(p, C_BRKPOLL, ST_NC);
        set_chk(p, C_BRKLATE, ST_NC);
    } else {
        RUN_CHK(p, C_BRKPOLL, break_test(base, 0));
        RUN_CHK(p, C_BRKLATE, break_test(base, 1));
    }
    RUN_CHK(p, C_THRE, thre_test(base));
    if (has_fifo)
        RUN_CHK(p, C_FCRSW, fcrsw_test(base));
    else
        set_chk(p, C_FCRSW, ST_NA);
    if (!p) {
        if (wd_lpt)
            RUN_CHK(0, C_TYPESW, typesw_test(base));
        else
            set_chk(0, C_TYPESW, ST_NA);
    }

    RUN_CHK(p, C_RTS, line_test(base, MCR_RTS, MSR_CTS, 8, NULL));
    RUN_CHK(p, C_DTR, line_test(base, MCR_DTR, MSR_DSR, 8, NULL));
    RUN_CHK(p, C_DCD, line_test(base, p ? MCR_DTR : MCR_OUT2, MSR_DCD, 4, NULL));
    if (p && no_ri)
        set_chk(p, C_RI, ST_NC);
    else
        RUN_CHK(p, C_RI, line_test(base, p ? MCR_DTR : MCR_OUT1, MSR_RI, 4, NULL));
    RUN_CHK(p, C_DELTA, delta_test(p));

    RUN_CHK(p, C_IRQRX, irq_check(irqm, IRQ_RX));
    RUN_CHK(p, C_IRQTHRE, irq_check(irqm, IRQ_THRE));
    RUN_CHK(p, C_IRQLSR, irq_check(irqm, IRQ_LSR));
    RUN_CHK(p, C_IRQMSR, irq_check(irqm, IRQ_MSR));
    if (has_fifo)
        RUN_CHK(p, C_IRQTMO, irq_check(irqm, IRQ_TMO));
    else
        set_chk(p, C_IRQTMO, ST_NA);

    uart_init(0x00);
}
