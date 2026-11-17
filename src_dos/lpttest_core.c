/*
 * lpttest_core.c -- the parallel port checks of LPTTEST, with no screen, keyboard,
 * file or operating-system code in it. Like sertest_core.c it is included into the
 * program's one translation unit (LPTTEST.EXE's lpttest.c, or a port such as the
 * Linux puppet's), after sertest_defs.h (machine layer, timing, note()).
 *
 * The program supplies:
 *   lpt_base              the port's base address (data), status at +1, control at +2
 *   lpt_set(check, state) a check's RUN/PASS/FAIL/SKIP/NA row (ST_* from sertest_defs.h)
 *   note(fmt, ...)        one failure detail line (sertest_defs.h)
 * and optionally lpt_irq, then calls lpt_run(). Every check works on a bare SPP port; the printer ones expect
 * an on-line printer behind the port (86Box: lpt1_device = text_prt). The bytes the
 * PRINT check sends are LPT_TEXT, so a host can compare the printer's output file.
 */

/* control register */
#define LC_STROBE  0x01     /* /STROBE, written 1 = line low (active) */
#define LC_AUTOFD  0x02
#define LC_INIT    0x04     /* 0 = reset the printer */
#define LC_SELIN   0x08
#define LC_IRQEN   0x10
#define LC_DIR     0x20     /* bidirectional ports: data pins are inputs */
/* status register */
#define LS_ERROR   0x08     /* 1 = no error */
#define LS_SELECT  0x10     /* 1 = on line */
#define LS_PE      0x20     /* 1 = out of paper */
#define LS_ACK     0x40     /* 0 = a character was just taken */
#define LS_BUSY    0x80     /* 0 = busy */

#define LPT_TEXT   "LPTTEST 86Box parallel port check\r\n0123456789ABCDEF\r\n\f"

enum { L_DATA, L_CTRL, L_LATCH, L_STATUS, L_PRINT, L_IRQ, L_INIT, NLCHK };

static const char *lpt_chk_name[NLCHK] = {
    "DATA", "CTRL", "LATCH", "STATUS", "PRINT", "IRQ", "INIT"
};
static const char *lpt_chk_text[NLCHK] = {
    "data register write/readback, 00-FF",
    "control register bits 0-4 write/readback",
    "data latch holds its value across control writes",
    "status register: printer idle, on line, no error",
    "strobe handshake: LPT_TEXT sent byte by byte",
    "printer ACK raises the port IRQ (control bit 4, PIC request register)",
    "/INIT pulse resets the printer and it comes back"
};

static unsigned int lpt_base;
static int lpt_irq = 7;              /* the port's IRQ (7 or 5 on a PC) */
static int lpt_sent;                 /* bytes PRINT handed over */

static void lpt_set(int check, int state);

#define LP_DATA   (lpt_base)
#define LP_STAT   (lpt_base + 1)
#define LP_CTRL   (lpt_base + 2)

static unsigned char lp_ctrl_idle(void)
{
    return LC_INIT | LC_SELIN;       /* not reset, selected, strobe off */
}

static void lp_idle(void)
{
    hw_out(LP_CTRL, lp_ctrl_idle());
}

static int lp_data_test(void)
{
    int v, bad = 0;
    lp_idle();
    for (v = 0; v < 256; v++) {
        unsigned char got;
        hw_out(LP_DATA, (unsigned char) v);
        got = (unsigned char) hw_in(LP_DATA);
        if (got != v) {
            if (bad++ < 3)
                note("data %02X wrote, read %02X", v, got);
        }
    }
    return bad == 0;
}

static int lp_ctrl_test(void)
{
    static const unsigned char pat[] = { 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x15, 0x0A, 0x1F, 0x0C };
    int i, bad = 0;
    for (i = 0; i < (int) sizeof(pat); i++) {
        unsigned char got;
        hw_out(LP_CTRL, pat[i]);
        got = (unsigned char) (hw_in(LP_CTRL) & 0x1F);
        if (got != pat[i]) {
            if (bad++ < 3)
                note("control %02X wrote, read %02X", pat[i], got);
        }
    }
    lp_idle();
    return bad == 0;
}

static int lp_latch_test(void)
{
    static const unsigned char pat[] = { 0x55, 0xAA, 0x01, 0x80 };
    int i, bad = 0;
    for (i = 0; i < (int) sizeof(pat); i++) {
        unsigned char c;
        hw_out(LP_DATA, pat[i]);
        for (c = 0; c < 0x20; c += 5)
            hw_out(LP_CTRL, (unsigned char) (c & ~LC_STROBE & ~LC_DIR));
        if ((unsigned char) hw_in(LP_DATA) != pat[i]) {
            if (bad++ < 3)
                note("latch %02X lost after control writes, read %02X", pat[i], hw_in(LP_DATA));
        }
    }
    lp_idle();
    return bad == 0;
}

static int lp_status_test(void)
{
    unsigned char s;
    lp_idle();
    t_sleep(TICKS_10MS);
    s = (unsigned char) hw_in(LP_STAT);
    if ((s & (LS_BUSY | LS_SELECT | LS_ERROR | LS_PE)) != (LS_BUSY | LS_SELECT | LS_ERROR)) {
        note("status %02X: want not busy, on line, no error, paper ok (D8-DF)", s);
        return 0;
    }
    return 1;
}

/* Wait until the printer is not busy. */
static int lp_wait_ready(unsigned long tmo)
{
    t_start();
    while (t_now() < tmo) {
        unsigned char s = (unsigned char) hw_in(LP_STAT);
        if (s & LS_BUSY)
            return 1;
    }
    return 0;
}

static int lp_print_test(void)
{
    const char *p = LPT_TEXT;
    int bad = 0;
    lpt_sent = 0;
    lp_idle();
    for (; *p; p++) {
        unsigned long hold;
        if (!lp_wait_ready(TICKS_1S)) {
            note("byte %d: printer stayed busy", lpt_sent);
            bad = 1;
            break;
        }
        hw_out(LP_DATA, (unsigned char) *p);
        hw_out(LP_CTRL, lp_ctrl_idle() | LC_STROBE);
        for (hold = 0; hold < 8; hold++)        /* >= 1 us of strobe */
            (void) hw_in(LP_STAT);
        hw_out(LP_CTRL, lp_ctrl_idle());
        lpt_sent++;
        /* the ACK pulse can come right away; poll it before the next byte */
        (void) lp_wait_ready(TICKS_100MS);
    }
    lp_idle();
    return !bad;
}

/* The ACK pulse is a few microseconds wide, too narrow to poll in the status register, so the check
 * is the interrupt it raises: IRQ enable on, one byte out, then the PIC's request register. User
 * space and DOS alike have no handler here, so a bit already set before the byte is stale: NA. */
static unsigned char lp_irr(void)
{
    hw_out(0x20, 0x0A);                 /* OCW3: next read of 0x20 is the IRR */
    return (unsigned char) hw_in(0x20);
}

static int lp_irq_test(void)
{
    unsigned char bit = (unsigned char) (1 << (lpt_irq & 7));
    if (lpt_irq > 7) {
        note("IRQ %d is on the slave PIC; not checked", lpt_irq);
        return -1;
    }
    hw_out(LP_CTRL, lp_ctrl_idle() | LC_IRQEN);
    if (lp_irr() & bit) {
        note("IRQ%d request already pending before the byte (stale), not checked", lpt_irq);
        lp_idle();
        return -1;
    }
    if (!lp_wait_ready(TICKS_1S)) {
        note("printer busy");
        lp_idle();
        return 0;
    }
    hw_out(LP_DATA, 0x20);
    hw_out(LP_CTRL, lp_ctrl_idle() | LC_IRQEN | LC_STROBE);
    hw_out(LP_CTRL, lp_ctrl_idle() | LC_IRQEN);
    t_start();
    while (t_now() < TICKS_10MS)
        if (lp_irr() & bit) {
            lp_idle();
            return 1;
        }
    note("no IRQ%d request within 10 ms of an ACK", lpt_irq);
    lp_idle();
    return 0;
}

static int lp_init_test(void)
{
    unsigned char s;
    hw_out(LP_CTRL, LC_SELIN);                  /* /INIT low */
    t_sleep(TICKS_1MS);
    hw_out(LP_CTRL, lp_ctrl_idle());
    t_sleep(TICKS_10MS);
    s = (unsigned char) hw_in(LP_STAT);
    if (!(s & LS_BUSY) || !(s & LS_SELECT)) {
        note("after /INIT status %02X: not ready again", s);
        return 0;
    }
    return 1;
}

#define LP_RUN(i, expr) do { lpt_set(i, ST_RUN); lpt_set(i, (expr) ? ST_PASS : ST_FAIL); } while (0)

static void lpt_run(void)
{
    LP_RUN(L_DATA, lp_data_test());
    LP_RUN(L_CTRL, lp_ctrl_test());
    LP_RUN(L_LATCH, lp_latch_test());
    LP_RUN(L_STATUS, lp_status_test());
    LP_RUN(L_PRINT, lp_print_test());
    {
        int r;
        lpt_set(L_IRQ, ST_RUN);
        r = lp_irq_test();
        lpt_set(L_IRQ, r < 0 ? ST_NA : r ? ST_PASS : ST_FAIL);
    }
    LP_RUN(L_INIT, lp_init_test());
}
