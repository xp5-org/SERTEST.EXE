/*
 * sertest.c : XP5 SERTEST, COM port loopback diagnostic.
 *
 * Needs 8088+, UART 8250/16450/16550/16550A, MSDOS 5.0+, 200K free.
 * PCjr untested.
 *
 * Every check runs on two panels:
 *   INTERNAL : UART diagnostic loopback, MCR bit 4. Tests serial.c alone.
 *   EXTERNAL : loopback plug on the connector, through the host char device.
 *              vuart_loopback.py on /dev/vuart0 on Linux, com0com on Windows.
 *
 * PAGE 1 FUNCTION : one PASS/FAIL per check and panel, at the top rate
 *   115200 or the /B: rate unless the check names one.
 *   data     scratch register, plug detect, 00-FF, all 40 word formats at the
 *            top rate and 2400, 16550 FIFO burst, overrun, THRE/TEMT, FCR
 *            mode switch
 *   break    LCR bit 6 held 40 character times at the top rate 8N1 and 2400
 *            7E1. A real UART gives one 0x00 with BI. Late read touches
 *            nothing until 300 ms after the break, a second 0x00 shows as OE.
 *   lines    RTS->CTS, DTR->DSR, DCD and RI, MSR delta bits, TERI when RI falls
 *   IRQ      one interrupt of each kind, IIR checked
 *   type sw  INT only, WD76C30 Amstrad MegaPC only, n/a elsewhere. Switches
 *            16450, 16550, 16450 and checks IIR 7:6 and how many of 4 bytes
 *            RBR holds.
 *   Failure notes fill the box below.
 *
 * PAGE 2 BAUD CYCLE : every rate 110-115200, 8N1, FIFO off, per panel.
 *   BURST   first byte from an idle line, THR write to LSR.DR, 8 samples
 *   RUN     per byte period of a long back-to-back run
 *   lines   RTS->CTS and DTR->DSR follow time, worst of 4 edges
 *   Times are in bit periods, one 8N1 character is 10. A real UART gives
 *   BURST about 10.5 and RUN 10.0. '*' flags a slow rate, BURST over 12,
 *   RUN over 10.3 or a line over 2 ms. 'X' flags a data error or timeout
 *   and fails the rate. Slow is reported, not failed. "cpu" means the
 *   system cannot poll the UART fast enough.
 *
 * PAGE 3 EXT LATENCY / PAGE 4 INT LATENCY : one panel each, in ms.
 *   rates   16 stop-and-wait round trips per rate. MIN AVG MAX JITTER cover
 *           the 14 bytes 01-FF. The two 0x00 are kept apart since a host
 *           port can hold a lone 0x00 back to check for a break. HOLD marks it.
 *   lines   16 edges per modem line at the top rate
 *   Each rate starts with an uncounted warm-up round trip, 0x55, then
 *   150 ms of RATE_SETTLE. SLOW is a round trip over 12 bits or a line over
 *   2 ms. SLOW and HOLD are reported. A timeout or wrong byte fails the row.
 *
 * PAGE 5 EXT SAMPLES / PAGE 6 INT SAMPLES : pages 3/4 samples in send order.
 *   Bright is over the SLOW bound, BAD is a wrong byte, TMO is a timeout.
 *
 * Usage:  SERTEST [/B:baud] [/COM:n] [/INT] [/EXT] [/NORI] [/NOBRK] [/STEP]
 *         SERTEST /LAT[:lines] [/B:baud] [/INT|/EXT] [/T:secs] [/COM:n] [/NORI]
 *         SERTEST /DUAL [/T:secs] [/NOBRK] [/STEP]
 *   /B:n    one rate instead of 110-115200
 *   /COM:n  port 1-4, base from the BIOS data area, IRQ 4 for 1/3 and 3 for 2/4
 *   /INT    internal panel only      /EXT  external panel only
 *   /NORI   plug does not wire RI, DTR->RI shows n/c
 *   /NOBRK  plug does not carry a break back, EXTERNAL break checks show n/c
 *   /LAT    latency only live screen, see run_latency. Lines are ALL or a list
 *           of DATA RTS DTR DCD RI for EXT and DATA RTS DTR OUT1 OUT2 for INT.
 *           Without /B it starts at 110 and Enter steps to the next rate.
 *   /T:n    latency pass seconds, default 2, per row in /DUAL
 *   /STEP   wait for G before each page, 1 2 4 3, and append
 *           "SERTEST PAGE n DONE" to the log, see step_gate
 *   /DUAL   COM1 and COM2 at once, log DUALLOG.TXT, see run_dual
 *
 * SERLOG.TXT beside SERTEST.EXE is overwritten every run and ends with
 * "SERTEST DONE RESULT=...". /LAT writes one line per pass.
 *
 * Page 3, or the last page run, stays up with "SERTEST RESULT: PASS|FAIL"
 * on the bottom line, RUNNING until everything has run. Keys 1-6 show a page,
 * Enter the next, R reruns the page, Esc exits with errorlevel 0 PASS or
 * 1 FAIL. Keys also work during the run, at the next item boundary.
 *
 * Build: wcl -0 -ms, OpenWatcom 16-bit real-mode DOS.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>
#include <dos.h>
#include <i86.h>
#include <stdarg.h>

#define REG_DATA  0
#define REG_IER   1
#define REG_DLL   0
#define REG_DLH   1
#define REG_IIR   2
#define REG_FCR   2
#define REG_LCR   3
#define REG_MCR   4
#define REG_LSR   5
#define REG_MSR   6
#define REG_SCR   7

#define LSR_DR    0x01
#define LSR_OE    0x02
#define LSR_BI    0x10
#define LSR_THRE  0x20
#define LSR_TEMT  0x40

#define MCR_DTR   0x01
#define MCR_RTS   0x02
#define MCR_OUT1  0x04
#define MCR_OUT2  0x08
#define MCR_LOOP  0x10

#define MSR_DCTS  0x01
#define MSR_DDSR  0x02
#define MSR_TERI  0x04
#define MSR_DDCD  0x08
#define MSR_CTS   0x10
#define MSR_DSR   0x20
#define MSR_RI    0x40
#define MSR_DCD   0x80

#define PIC1_CMD  0x20
#define PIC1_DATA 0x21

#define PIT_CH0   0x40
#define PIT_CMD   0x43
#define PIT_HZ    1193182UL

#define TICKS_1MS    1193UL
#define TICKS_10MS   11932UL
#define TICKS_100MS  119318UL
#define TICKS_1S     1193182UL
#define TMO_TICKS    TICKS_1S   /* per-sample timeout */


#define NCHK      22        /* page 1 checks: column 1 holds 11, column 2 the rest */
#define NCOL1     11
#define NBAUD     12
#define BI_2400   5         /* baud_tab index of 2400 */
#define NPAGE     6

#define P1_FIRST  3         /* page 1: first check row */
#define NOTE_TOP  14        /* page 1 failure-notes box */
#define NOTE_ROWS 6
#define LEG_ROW   22        /* one-line legend under a page's box */
#define STAGE_ROW 23        /* stage / progress line, run counts when done */
#define FOOT_ROW  24        /* keys and SERTEST RESULT: */

#define A_NORM    0x07
#define A_BRIGHT  0x0F

/* Row states */
#define ST_PEND   0
#define ST_RUN    1
#define ST_PASS   2
#define ST_FAIL   3
#define ST_SKIP   4         /* panel not run */
#define ST_NC     5         /* not wired on this plug */
#define ST_NA     6         /* check doesn't apply to this panel or UART */

static const unsigned long baud_tab[NBAUD] = {
    110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200UL
};
static const unsigned int div_tab[NBAUD] = {
    1047, 768, 384, 192, 96, 48, 24, 12, 6, 3, 2, 1
};
/* Rate labels for page 2's RATE column: 5 chars max. */
static const char *rate_lbl[NBAUD] = {
    "110", "150", "300", "600", "1200", "2400", "4800", "9600", "19.2k", "38.4k", "57.6k", "115k"
};
static int nbaud = NBAUD, baud0 = 0;   /* active slice of baud_tab */
static int cur_bi = NBAUD - 1;
static unsigned char cur_lcr = 0x03;

/* Bit patterns every sweep sends, masked to the word length in use. */
static const unsigned char pat_tab[8] = { 0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x01, 0x80 };


/* Page 1 checks. Column 1 is the data path, column 2 the modem
 * lines and interrupts. panels: 1 INT, 2 EXT, 3 both. */
enum {
    C_SCRATCH, C_PLUG, C_DATA, C_FMT, C_FMT2400, C_BURST, C_OVERRUN, C_BRKPOLL, C_BRKLATE, C_THRE, C_FCRSW,
    C_RTS, C_DTR, C_DCD, C_RI, C_DELTA, C_IRQRX, C_IRQTHRE, C_IRQLSR, C_IRQMSR, C_IRQTMO,
    C_TYPESW
};

struct chk_def {
    const char   *name;         /* page 1 row, 24 chars at most */
    const char   *stage;        /* stage line and SERLOG tag */
    unsigned char panels;
};

static char fmt_top_name[24] = "Formats x40 @115200";

static const struct chk_def chk[NCHK] = {
    { "Scratch register R/W",   "SCRATCH",  1 },
    { "Plug detect",            "PLUG",     2 },
    { "Data 00-FF  TX->RX",     "DATA",     3 },
    { fmt_top_name,             "FORMAT",   3 },
    { "Formats x40 @2400",      "FMT2400",  3 },
    { "FIFO burst, 4 formats",  "FIFO",     3 },
    { "Overrun -> OE",          "OVERRUN",  3 },
    { "Break -> BI, polled",    "BRK-POLL", 3 },
    { "Break -> BI, late read", "BRK-LATE", 3 },
    { "THRE/TEMT",              "THRE",     3 },
    { "FCR mode switch",        "FCRSW",    3 },
    { "RTS -> CTS",             "RTS-CTS",  3 },
    { "DTR -> DSR",             "DTR-DSR",  3 },
    { "OUT2|DTR -> DCD",        "DCD",      3 },
    { "OUT1|DTR -> RI",         "RI",       3 },
    { "MSR deltas + TERI",      "DELTA",    3 },
    { "IRQ RX data",            "IRQ-RX",   3 },
    { "IRQ THR empty",          "IRQ-THRE", 3 },
    { "IRQ line status (OE)",   "IRQ-LSR",  3 },
    { "IRQ modem status",       "IRQ-MSR",  3 },
    { "IRQ RX FIFO timeout",    "IRQ-TMO",  3 },
    { "16550->16450 type sw",   "TYPESW",   1 }
};

/* Failure details for SERLOG.TXT and page 1's notes box. Capped so one
 * broken rate cannot flood the log. */
#define NOTE_MAX 48
#define NOTE_LEN 96                     /* the log line; the screen shows 80 */
static char notes[NOTE_MAX][NOTE_LEN];
static unsigned char note_pg[NOTE_MAX]; /* page that wrote each note */
static int  nnotes, notes_dropped;
static const char *note_panel = "";
static int  cur_run_pg;                 /* page now running, tags new notes */

static void note(const char *fmt, ...)
{
    static char buf[256];
    va_list ap;
    int n;
    if (nnotes == NOTE_MAX) {
        notes_dropped++;
        return;
    }
    note_pg[nnotes] = (unsigned char) cur_run_pg;
    n = sprintf(buf, "%s ", note_panel);
    va_start(ap, fmt);
    vsprintf(buf + n, fmt, ap);         /* callers keep it under 250 */
    va_end(ap);
    /* keep a long note out of the next slot */
    strncpy(notes[nnotes], buf, NOTE_LEN - 1);
    notes[nnotes][NOTE_LEN - 1] = '\0';
    nnotes++;
}

/* A page rerun starts with none of that page's old notes. */
static void notes_drop_pg(int pg)
{
    int i, j = 0;
    for (i = 0; i < nnotes; i++)
        if (note_pg[i] != pg) {
            if (j != i) {
                strcpy(notes[j], notes[i]);
                note_pg[j] = note_pg[i];
            }
            j++;
        }
    nnotes = j;
    notes_dropped = 0;
}

static const char *prog_path = "";
static int chk_st[2][NCHK];             /* [0 INT, 1 EXT][check] */

static unsigned int  port;
static int           com_n = 1;
static int           irq;
static int           do_int = 1, do_ext = 1, no_ri = 0, no_brk = 0;
static int           lat_mode = 0;          /* /LAT: live latency-only screen */
static int           step_mode = 0;         /* /STEP: each stage waits for G */
static int           dual_mode = 0;         /* /DUAL: COM1 + COM2 at once */
static char          lat_arg[40] = "ALL";
static int           lat_secs = 2;          /* /T: pass length */
static int           ext_noplug;
static const char   *uart_name = "?";
static int           has_fifo;              /* 16550A: working FIFOs */
static int           has_scr;               /* 16450 and up: scratch register */
static unsigned int  wd_lpt;                /* LPT1 base if the type switch answered */
static unsigned char wd_sel, wd_r5;         /* WD76C30 index and register 5 as found */
static const char   *cpu_name = "?";
static unsigned long poll16;                /* one idle poll-loop pass, 1/16 PIT ticks */
static int           cpu_top = 11;          /* fastest rate the CPU can poll */
static unsigned char far *vram;

static volatile unsigned int  isr_hits;
static volatile int           isr_data;
static volatile unsigned char isr_iir;
static volatile unsigned char isr_lsr;
static void (__interrupt __far *old_isr)(void);

/* ------------------------------------------------------------------ PIT */

static unsigned int  t_last;
static unsigned long t_acc;

static unsigned int pit_read(void)
{
    unsigned char lo, hi;
    _disable();
    outp(PIT_CMD, 0x00);            /* latch channel 0 */
    lo = (unsigned char) inp(PIT_CH0);
    hi = (unsigned char) inp(PIT_CH0);
    _enable();
    return ((unsigned int) hi << 8) | lo;
}

static void pit_mode(unsigned char mode_cmd)
{
    _disable();
    outp(PIT_CMD, mode_cmd);        /* ch0, lobyte/hibyte, mode 2 0x34 or 3 0x36 */
    outp(PIT_CH0, 0);
    outp(PIT_CH0, 0);               /* reload 0 == 65536, the BIOS's own 18.2Hz */
    _enable();
}

/* Every PIT read also advances the latency pass clock p_acc. */
static unsigned long p_acc;

static unsigned int pit_step(void)
{
    unsigned int c = pit_read(), d = (unsigned int) (t_last - c);
    t_last = c;
    p_acc += d;
    return d;
}

static void clk_init(void)
{
    t_last = pit_read();
    t_acc = p_acc = 0;
}

static void t_start(void)
{
    (void) pit_step();
    t_acc = 0;
}

/* Elapsed ticks since t_start. The counter counts down and wraps every
 * 65536 ticks, so poll more often than that. */
static unsigned long t_now(void)
{
    t_acc += pit_step();
    return t_acc;
}

/* Run clock for latency mode: BIOS tick count at 0040:006C. The PIT
 * clocks would lose wraps during floppy log writes. */
static unsigned long run_t0;

static unsigned long bios_ticks(void)
{
    unsigned long far *t = (unsigned long far *) MK_FP(0x0040, 0x006C);
    unsigned long v;
    _disable();
    v = *t;
    _enable();
    return v;
}

static unsigned long run_secs(void)
{
    return (bios_ticks() - run_t0) * 100UL / 1821UL;
}

static void t_sleep(unsigned long ticks)
{
    t_start();
    while (t_now() < ticks)
        ;
}

/* Wait after a rate change before timing. An emulator catches up after
 * the host port reconfig and a guest wait in that window reads long. */
#define RATE_SETTLE (150UL * TICKS_1MS)

/* Keys during a run. Sub-tests poll between items and stop the page at
 * the next boundary. Esc leaves the program, R reruns the page. */
#define STOP_NONE   0
#define STOP_QUIT   1
#define STOP_RERUN  2
static int stop_req;

static void key_poll(void)
{
    int k;
    while (kbhit()) {
        k = getch();
        if (k == 0 || k == 0xE0) {
            (void) getch();             /* extended key: ignore */
        } else if (k == 27) {
            stop_req = STOP_QUIT;
        } else if ((k == 'r' || k == 'R') && stop_req != STOP_QUIT) {
            stop_req = STOP_RERUN;
        }
    }
}

/* t_sleep that notices keys, for the pauses between pages. */
static void t_sleep_keys(unsigned long ticks)
{
    t_start();
    while (t_now() < ticks && !stop_req)
        key_poll();
}

/* ---------------------------------------------------------------- video */

static void vput(int r, int c, unsigned char a, const char *s)
{
    unsigned char far *p = vram + (r * 80 + c) * 2;
    while (*s && c < 80) {
        p[0] = (unsigned char) *s++;
        p[1] = a;
        p += 2;
        c++;
    }
}

static void vfill(int r, int c, int n, unsigned char ch, unsigned char a)
{
    unsigned char far *p = vram + (r * 80 + c) * 2;
    while (n-- > 0 && c++ < 80) {
        p[0] = ch;
        p[1] = a;
        p += 2;
    }
}

static void vch(int r, int c, unsigned char ch, unsigned char a)
{
    vfill(r, c, 1, ch, a);
}

static void cursor(int on)
{
    union REGS rg;
    rg.h.ah = 0x01;
    rg.x.cx = on ? (vram == (unsigned char far *) MK_FP(0xB000, 0) ? 0x0B0C : 0x0607) : 0x2000;
    int86(0x10, &rg, &rg);
    if (on) {
        rg.h.ah = 0x02;
        rg.h.bh = 0;
        rg.x.dx = 0;
        int86(0x10, &rg, &rg);
    }
}

static void cls(void)
{
    int r;
    for (r = 0; r < 25; r++)
        vfill(r, 0, 80, ' ', A_NORM);
}

/* Double-line panel, 40 columns at column c, title in the top border on
 * row top, then rows content rows. */
static void draw_frame(int c, int top, int rows, const char *title)
{
    int r;
    vch(top, c, 0xC9, A_NORM);
    vfill(top, c + 1, 38, 0xCD, A_NORM);
    vch(top, c + 39, 0xBB, A_NORM);
    vch(top, c + 2, ' ', A_NORM);
    vput(top, c + 3, A_NORM, title);
    vch(top, c + 3 + (int) strlen(title), ' ', A_NORM);
    for (r = top + 1; r <= top + rows; r++) {
        vch(r, c, 0xBA, A_NORM);
        vfill(r, c + 1, 38, ' ', A_NORM);
        vch(r, c + 39, 0xBA, A_NORM);
    }
    vch(top + rows + 1, c, 0xC8, A_NORM);
    vfill(top + rows + 1, c + 1, 38, 0xCD, A_NORM);
    vch(top + rows + 1, c + 39, 0xBC, A_NORM);
}

static const char *panel_title[2] = { "INTERNAL  (MCR bit4 loop)", "EXTERNAL  (plug on connector)" };

static const char *page_name[NPAGE] = { "FUNCTION", "BAUD CYCLE", "EXT LATENCY", "INT LATENCY",
                                         "EXT SAMPLES", "INT SAMPLES" };

/* Bottom line on every page: the keys, then RUNNING, PASS or FAIL. The
 * testlists OCR the RESULT text. */
static const char *result_word = "RUNNING";

static void draw_footer(void)
{
    vfill(FOOT_ROW, 0, 80, ' ', A_NORM);
    if (step_mode && result_word[0] == 'R')
        vput(FOOT_ROW, 1, A_NORM, "G=run next stage  Esc=exit");
    else if (dual_mode)
        vput(FOOT_ROW, 1, A_NORM, "1-2=page  Enter=next  Esc=exit");
    else
        vput(FOOT_ROW, 1, A_NORM, "1-6=page  R=rerun page  Enter=next  Esc=exit");
    vput(FOOT_ROW, 55, A_NORM, "SERTEST RESULT: ");
    vput(FOOT_ROW, 71, result_word[0] == 'F' ? A_BRIGHT : A_NORM, result_word);
}

/* Row 0 on every page: same fields in the same place, only the page differs. */
static void draw_header(int page)
{
    char buf[81];
    cls();
    sprintf(buf, " XP5 SERTEST 0.1  PAGE %d OF %d %-11s COM%d %03Xh IRQ%d UART %-6s CPU %s",
            page, NPAGE, page_name[page - 1], com_n, port, irq, uart_name, cpu_name);
    vput(0, 0, A_NORM, buf);
    draw_footer();
}

/* Single-line boxes, full width. A title is set into the border as
 * "-- TITLE ---". */
static void box_title(int r, const char *title)
{
    vch(r, 2, ' ', A_NORM);
    vput(r, 3, A_NORM, title);
    vch(r, 3 + (int) strlen(title), ' ', A_NORM);
}

static void box(int top, int bot, const char *title)
{
    int r;
    vch(top, 0, 0xDA, A_NORM);
    vfill(top, 1, 78, 0xC4, A_NORM);
    vch(top, 79, 0xBF, A_NORM);
    for (r = top + 1; r < bot; r++) {
        vch(r, 0, 0xB3, A_NORM);
        vfill(r, 1, 78, ' ', A_NORM);
        vch(r, 79, 0xB3, A_NORM);
    }
    vch(bot, 0, 0xC0, A_NORM);
    vfill(bot, 1, 78, 0xC4, A_NORM);
    vch(bot, 79, 0xD9, A_NORM);
    box_title(top, title);
}

/* A rule across a box at row r, titled if title is not empty. */
static void box_rule(int r, const char *title)
{
    vch(r, 0, 0xC3, A_NORM);
    vfill(r, 1, 78, 0xC4, A_NORM);
    vch(r, 79, 0xB4, A_NORM);
    if (title[0])
        box_title(r, title);
}

/* A divider down column c from row r0 to r1, joined to the rows r0 - 1
 * and r1 + 1 with the given corner characters. */
static void box_split(int c, int r0, int r1, unsigned char top, unsigned char bot)
{
    int r;
    vch(r0 - 1, c, top, A_NORM);
    for (r = r0; r <= r1; r++)
        vch(r, c, 0xB3, A_NORM);
    vch(r1 + 1, c, bot, A_NORM);
}

/* Stage line above the footer: "<stage> - TESTING" at start and
 * "<stage> - PASS|FAIL" at the end. The right half keeps the previous
 * verdict. The run counts replace it when everything is done. */
static char stage_prev[40];
static char stage_cur[32];
static int  prog_last = -1;

static void stage(const char *name, const char *state)
{
    char line[48];
    strncpy(stage_cur, name, sizeof(stage_cur) - 1);
    stage_cur[sizeof(stage_cur) - 1] = 0;
    prog_last = -1;
    sprintf(line, "%s - %s", name, state);
    vfill(STAGE_ROW, 0, 80, ' ', A_NORM);
    vput(STAGE_ROW, 1, A_BRIGHT, line);
    if (stage_prev[0])
        vput(STAGE_ROW, 42, A_NORM, stage_prev);
    if (state[0] != 'T')                  /* TESTING is not a verdict */
        strcpy(stage_prev, line);
}

/* Progress in a long stage, in the left 40 columns of the stage line.
 * Redrawn about 25 times per stage. */
static void progress(int done, int total)
{
    char line[48];
    int step = total / 25;
    if (step < 1)
        step = 1;
    if (done != 0 && done < total && done - prog_last < step)
        return;
    prog_last = done;
    sprintf(line, "%s - TESTING %d/%d", stage_cur, done, total);
    vfill(STAGE_ROW, 1, 40, ' ', A_NORM);
    vput(STAGE_ROW, 1, A_BRIGHT, line);
}

static const char *st_word[] = { "", "TESTING", "PASS", "FAIL", "", "", "" };
static int cur_page;                   /* page on screen: rows only draw on their own page */

static void stage_row(const char *panel, const char *tag, int st)
{
    char name[24];
    if (st != ST_RUN && st != ST_PASS && st != ST_FAIL)
        return;
    sprintf(name, "%s %s", panel, tag);
    stage(name, st_word[st]);
}

static int top_bi(void);
static void draw_notes(void);

/* Check i's row on page 1. Column 1 holds checks 0-9, column 2 10-19. */
static void draw_chk(int i)
{
    static const char *txt[] = { "    ", "RUN ", "PASS", "FAIL", "----", "n/c ", " -- " };
    int r = P1_FIRST + i % NCOL1, c = (i < NCOL1) ? 1 : 41, n = (int) strlen(chk[i].name), p, st;

    vput(r, c + 1, A_NORM, chk[i].name);
    vch(r, c + 1 + n, ' ', A_NORM);
    vfill(r, c + 2 + n, 24 - n, '.', A_NORM);
    for (p = 0; p < 2; p++) {
        st = chk_st[p][i];
        vput(r, c + 27 + p * 6, (st == ST_FAIL || st == ST_RUN) ? A_BRIGHT : A_NORM, txt[st]);
    }
}

static void set_chk(int p, int i, int st)
{
    chk_st[p][i] = st;
    if (cur_page == 1) {
        draw_chk(i);
        if (st == ST_FAIL)
            draw_notes();
    }
    stage_row(p ? "EXT" : "INT", chk[i].stage, st);
}

static void fmt_ms(char *buf, unsigned long t, int tmo)
{
    unsigned long ms10 = t * 100UL / 11932UL;   /* tenths of a millisecond */
    if (tmo)
        strcpy(buf, "TMO");
    else if (ms10 < 1)
        strcpy(buf, "<0.1 ms");
    else if (ms10 < 100)
        sprintf(buf, "%lu.%lu ms", ms10 / 10, ms10 % 10);
    else
        sprintf(buf, "%lu ms", ms10 / 10);
}

static int bucket(unsigned long t)
{
    if (t < TICKS_1MS)   return 0;
    if (t < TICKS_10MS)  return 1;
    if (t < TICKS_100MS) return 2;
    if (t < TICKS_1S)    return 3;
    return 4;
}

static void status(const char *s)
{
    vfill(STAGE_ROW, 0, 80, ' ', A_NORM);
    vput(STAGE_ROW, 1, A_NORM, s);
}

/* A notes box under the rule on row top, the first rows failure notes
 * and the rest counted. */
static const char *log_name = "SERLOG.TXT";     /* the log file; /DUAL: DUALLOG.TXT */

static void draw_notes_at(int top, int rows)
{
    char buf[80];
    int i, more = nnotes - rows + notes_dropped;

    for (i = 0; i < rows; i++)
        vfill(top + 1 + i, 1, 78, ' ', A_NORM);
    if (nnotes == 0)
        vput(top + 1, 2, A_NORM, "none");
    for (i = 0; i < nnotes && i < rows; i++) {
        if (i == rows - 1 && more > 0) {
            sprintf(buf, "... %d more in %s", more + 1, log_name);
            vput(top + 1 + i, 2, A_NORM, buf);
        } else
            vput(top + 1 + i, 2, A_NORM, notes[i]);
    }
}

static void draw_notes(void)
{
    draw_notes_at(NOTE_TOP, NOTE_ROWS);
}

/* fresh=1 starts a run with all rows pending, fresh=0 repaints the
 * results collected. */
static void draw_page1(int fresh)
{
    char buf[81];
    int i, p;

    cur_page = 1;
    draw_header(1);
    sprintf(fmt_top_name, "Formats x40 @%lu", baud_tab[top_bi()]);
    sprintf(buf, "FUNCTION \xC4\xC4 checks at %lu 8N1", baud_tab[top_bi()]);
    box(1, NOTE_TOP + NOTE_ROWS + 1, buf);
    box_rule(NOTE_TOP, "FAILURE NOTES");
    box_split(40, 2, NOTE_TOP - 1, 0xC2, 0xC1);
    for (p = 0; p < 2; p++) {
        vput(2, p * 40 + 2, A_NORM, "CHECK");
        vput(2, p * 40 + 28, A_NORM, "INT");
        vput(2, p * 40 + 34, A_NORM, "EXT");
    }
    for (i = 0; i < NCHK; i++) {
        if (fresh)
            for (p = 0; p < 2; p++)
                chk_st[p][i] = !(chk[i].panels & (1 << p)) ? ST_NA
                             : (p ? do_ext : do_int) ? ST_PEND : ST_SKIP;
        draw_chk(i);
    }
    vput(LEG_ROW, 1, A_NORM, "Break: 40 char times. Late read: one LSR read 300 ms after break end.");
    if (!fresh)
        draw_notes();
}

/* ----------------------------------------------------------------- UART */

static void uart_init(unsigned char mcr)
{
    unsigned int div = div_tab[cur_bi];
    outp(port + REG_IER, 0x00);
    outp(port + REG_LCR, 0x80);
    outp(port + REG_DLL, div & 0xFF);
    outp(port + REG_DLH, div >> 8);
    outp(port + REG_LCR, cur_lcr);
    outp(port + REG_FCR, 0x00);     /* FIFO off: strict per-byte behavior */
    outp(port + REG_MCR, mcr);
    (void) inp(port + REG_LSR);
    (void) inp(port + REG_MSR);
    (void) inp(port + REG_IIR);
}

static void drain_rx(void)
{
    int n = 0;
    while ((inp(port + REG_LSR) & LSR_DR) && n++ < 64)
        (void) inp(port + REG_DATA);
}

static void detect_uart(void)
{
    unsigned char iir;
    outp(port + REG_FCR, 0xC7);
    iir = (unsigned char) (inp(port + REG_IIR) & 0xC0);
    outp(port + REG_FCR, 0x00);
    has_fifo = (iir == 0xC0);
    has_scr = 1;
    if (iir == 0xC0)
        uart_name = "16550A";
    else if (iir == 0x80)
        uart_name = "16550";
    else {
        outp(port + REG_SCR, 0x5A);
        has_scr = (inp(port + REG_SCR) == 0x5A);
        uart_name = has_scr ? "16450" : "8250";
    }
}

/* ---- CPU class from the FLAGS bits each generation lets software change.
 * An 8086 reads FLAGS 12-15 as 1, a 286 as 0, a 386 cannot set EFLAGS.AC,
 * a 486 cannot set ID, otherwise CPUID gives the family. The 32-bit parts
 * are hand-assembled bytes since the build is wcl -0. */
extern unsigned int flags_get(void);
#pragma aux flags_get = "pushf" "pop ax" value [ax] modify exact [ax];

/* FLAGS = f, read it back, restore the old FLAGS. */
extern unsigned int flags_try(unsigned int f);
#pragma aux flags_try = "pushf" "push ax" "popf" "pushf" "pop ax" "popf" \
    parm [ax] value [ax] modify exact [ax];

/* EFLAGS AC and ID toggled, read back, restored. Returns the high word
 * of the changed bits. */
extern unsigned int eflags_flip(void);
#pragma aux eflags_flip = \
    0x66 0x9C                       /* pushfd */ \
    0x66 0x58                       /* pop eax */ \
    0x66 0x8B 0xC8                  /* mov ecx, eax */ \
    0x66 0x35 0x00 0x00 0x24 0x00   /* xor eax, 240000h */ \
    0x66 0x50                       /* push eax */ \
    0x66 0x9D                       /* popfd */ \
    0x66 0x9C                       /* pushfd */ \
    0x66 0x58                       /* pop eax */ \
    0x66 0x51                       /* push ecx */ \
    0x66 0x9D                       /* popfd */ \
    0x66 0x33 0xC1                  /* xor eax, ecx */ \
    0x66 0xC1 0xE8 0x10             /* shr eax, 16 */ \
    value [ax] modify exact [ax cx];

/* CPUID leaf 1: the family field. */
extern unsigned int cpuid_family(void);
#pragma aux cpuid_family = \
    0x66 0xB8 0x01 0x00 0x00 0x00   /* mov eax, 1 */ \
    0x0F 0xA2                       /* cpuid */ \
    0x66 0xC1 0xE8 0x08             /* shr eax, 8 */ \
    0x25 0x0F 0x00                  /* and ax, 0Fh */ \
    value [ax] modify exact [ax bx cx dx];

static void detect_cpu(void)
{
    unsigned int f = flags_get(), hi;

    if ((flags_try(f & 0x0FFF) & 0xF000) == 0xF000)
        cpu_name = "8086/88";
    else if ((flags_try(f | 0xF000) & 0xF000) == 0)
        cpu_name = "286";
    else {
        hi = eflags_flip();
        if (!(hi & 0x0004))
            cpu_name = "386";
        else if (!(hi & 0x0020))
            cpu_name = "486";
        else {
            f = cpuid_family();
            cpu_name = f == 4 ? "486" : f == 5 ? "P5" : "P6+";
        }
    }
}

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
        l = (unsigned char) inp(port + REG_LSR);
        t = t_now();
        if (l & LSR_DR)
            (void) inp(port + REG_DATA);
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
    while (!(inp(port + REG_LSR) & LSR_THRE))
        if (t_now() >= TMO_TICKS)
            return 0;
    drain_rx();
    t_start();
    outp(port + REG_DATA, b);
    while (!((l = (unsigned char) inp(port + REG_LSR)) & LSR_DR))
        if (t_now() >= TMO_TICKS)
            return 0;
    *ticks = t_now();
    last_lsr = l;
    *got = (unsigned char) inp(port + REG_DATA);
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
    outp(port + REG_FCR, 0x07);         /* FIFOs on, both cleared, RX trigger 1 */
    drain_rx();
    rnd_state = seed;
    for (i = 0; i < n; i++)
        burst_buf[i] = (unsigned char) (rnd8() & mask);

    t_start();
    while (recv < n) {
        lsr = (unsigned char) inp(port + REG_LSR);
        err |= (unsigned char) (lsr & 0x8E);   /* OE PE FE, RX FIFO error */
        if (lsr & LSR_DR) {
            c = (unsigned char) inp(port + REG_DATA);
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
                outp(port + REG_DATA, burst_buf[sent++]);
        if (t_now() - last > TMO_TICKS)          /* 1s with no byte back */
            break;
    }
    /* anything still arriving after the burst is a duplicate */
    t_sleep(4 * char_ticks);
    while ((inp(port + REG_LSR) & LSR_DR) && extra < 64) {
        (void) inp(port + REG_DATA);
        extra++;
    }
    outp(port + REG_FCR, 0x00);

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
        if ((inp(port + REG_MSR) & mask) == want) {
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
        outp(port + REG_MCR, rising ? base : (base | obit));
        t_start();
        if (!wait_msr(ibit, rising ? 0 : ibit, &t)) {
            ok = 0;                          /* never settled to the start state */
            t = TMO_TICKS;
        } else {
            t_start();
            outp(port + REG_MCR, rising ? (base | obit) : base);
            if (!wait_msr(ibit, rising ? ibit : 0, &t)) {
                ok = 0;
                t = TMO_TICKS;
            }
        }
        if (worst && t > *worst)
            *worst = t;
    }
    outp(port + REG_MCR, base);
    return ok;
}

/* One interrupt per check. The first records IIR and sets IER to 0, then
 * each handler clears what it raised. */
static void __interrupt __far com_isr(void)
{
    unsigned char iir = (unsigned char) inp(port + REG_IIR);
    if (isr_hits++ == 0) {
        isr_iir = iir;
        outp(port + REG_IER, 0x00);
    }
    isr_lsr |= (unsigned char) inp(port + REG_LSR);
    while (inp(port + REG_LSR) & LSR_DR)
        isr_data = inp(port + REG_DATA);
    (void) inp(port + REG_MSR);
    outp(PIC1_CMD, 0x20);
}

static int wait_thre(void)
{
    t_start();
    while (!(inp(port + REG_LSR) & LSR_THRE))
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
    (void) inp(port + REG_LSR);
    outp(port + REG_DATA, 0x11);
    if (!wait_thre())
        return 0;
    outp(port + REG_DATA, 0x22);
    if (!wait_thre())
        return 0;
    outp(port + REG_DATA, 0x33);
    t_sleep(20 * ch + TICKS_100MS / 2);
    l = (unsigned char) inp(port + REG_LSR);
    d = (unsigned char) inp(port + REG_DATA);
    l2 = (unsigned char) inp(port + REG_LSR);
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
    (void) inp(port + REG_LSR);

    outp(port + REG_LCR, (unsigned char) (lcr | 0x40));
    if (late) {
        t_sleep(hold);
        outp(port + REG_LCR, lcr);
        t_sleep(settle);
        l = (unsigned char) inp(port + REG_LSR);
        if (l & LSR_DR)
            d = (unsigned char) inp(port + REG_DATA);
        l2 = (unsigned char) inp(port + REG_LSR);
        ok = (l & (LSR_DR | LSR_OE | LSR_BI)) == (LSR_DR | LSR_BI) && d == 0x00 && !(l2 & LSR_DR);
        if (!ok)
            note("brk %lu %s late: LSR %02X RBR %02X, then LSR %02X (want 11+ 00, no DR)",
                 baud_tab[bi], fn, l, d, l2);
    } else {
        t_start();
        for (;;) {
            t = t_now();
            if (!off && t >= hold) {
                outp(port + REG_LCR, lcr);
                off = 1;
                t_start();
                continue;
            }
            if (off && t >= settle)
                break;
            l = (unsigned char) inp(port + REG_LSR);
            if (l & LSR_DR) {
                c = (unsigned char) inp(port + REG_DATA);
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
        outp(port + REG_SCR, pat[i]);
        if ((unsigned char) inp(port + REG_SCR) != pat[i])
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
    while ((inp(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT) && t_now() < TMO_TICKS)
        ;
    outp(port + REG_DATA, 0x3C);
    l = (unsigned char) inp(port + REG_LSR);
    if (!(l & LSR_TEMT)) {
        t_start();
        while ((inp(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT) && t_now() < TMO_TICKS)
            ;
        ok = (inp(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) == (LSR_THRE | LSR_TEMT);
    }
    t_sleep(TICKS_10MS);
    drain_rx();
    return ok;
}

/* LSR & mask == want within TMO_TICKS. */
static int wait_lsr(unsigned char mask, unsigned char want)
{
    t_start();
    while ((inp(port + REG_LSR) & mask) != want && t_now() < TMO_TICKS)
        ;
    return (inp(port + REG_LSR) & mask) == want;
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
    outp(port + REG_DATA, 0x3C);
    outp(port + REG_FCR, 0x01);
    if (!wait_lsr(idle, idle)) {
        note("fcrsw off->on, byte in THR: THRE/TEMT stuck, LSR %02X", inp(port + REG_LSR));
        ok = 0;
    }
    t_sleep(TICKS_100MS);
    drain_rx();

    uart_init(mcr);
    (void) wait_lsr(idle, idle);
    outp(port + REG_FCR, 0x07);
    for (i = 0; i < 4; i++)
        outp(port + REG_DATA, (unsigned char) (0xA0 + i));
    outp(port + REG_FCR, 0x00);
    if (!wait_lsr(idle, idle)) {
        note("fcrsw on->off, bytes queued: THRE/TEMT stuck, LSR %02X", inp(port + REG_LSR));
        ok = 0;
    }
    t_sleep(TICKS_100MS);
    drain_rx();

    for (m = 0; m < 2; m++) {
        uart_init(mcr);
        (void) wait_lsr(idle, idle);
        if (m)
            outp(port + REG_FCR, 0x07);
        outp(port + REG_DATA, 0x5A);
        if (!wait_lsr(LSR_DR, LSR_DR)) {
            note("fcrsw %s: no DR before the switch", m ? "on->off" : "off->on");
            ok = 0;
            continue;
        }
        outp(port + REG_FCR, m ? 0x00 : 0x01);
        l = (unsigned char) inp(port + REG_LSR);
        if (l & (LSR_DR | 0x80)) {
            note("fcrsw %s, byte waiting: LSR %02X after the switch (want DR and bit 7 clear)",
                 m ? "on->off" : "off->on", l);
            ok = 0;
        }
        t_sleep(TICKS_100MS);
        drain_rx();
    }

    outp(port + REG_FCR, 0x00);
    return ok;
}

/* ---- WD76C30 UART type switch, Amstrad MegaPC. Index at LPT base + 3,
 * data at LPT base + 7. Register 5 bit 0 is 1 for 16550 and 0 for 16450.
 * Change it read-modify-write and keep the index clock bits. */
static void wd_set(unsigned char r5)
{
    outp(wd_lpt + 3, (wd_sel & 0xF8) | 0x05);
    outp(wd_lpt + 7, r5);
}

static void wd_restore(void)
{
    wd_set(wd_r5);
    outp(wd_lpt + 3, wd_sel);
}

/* Only on a UART that reads as 16450 with an LPT1. Setting register 5
 * bit 0 must give the UART a FIFO, IIR 7:6 = C0. FCR=00 before switching
 * back. */
static void detect_typesw(void)
{
    unsigned int lpt = *(unsigned int far *) MK_FP(0x0040, 0x0008);
    unsigned char iir;

    wd_lpt = 0;
    if (has_fifo || !has_scr || !lpt)
        return;
    wd_sel = (unsigned char) inp(lpt + 3);
    outp(lpt + 3, (wd_sel & 0xF8) | 0x05);
    if ((unsigned char) inp(lpt + 3) != ((wd_sel & 0xF8) | 0x05)) {
        outp(lpt + 3, wd_sel);
        return;
    }
    wd_r5 = (unsigned char) inp(lpt + 7);
    wd_lpt = lpt;
    wd_set((unsigned char) (wd_r5 | 0x01));
    outp(port + REG_FCR, 0x07);
    iir = (unsigned char) (inp(port + REG_IIR) & 0xC0);
    outp(port + REG_FCR, 0x00);
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
    (void) inp(port + REG_LSR);
    for (i = 0; i < TYPESW_N; i++) {
        if (i && !wait_thre())
            return;
        outp(port + REG_DATA, (unsigned char) (0x41 + i));
    }
    t_sleep(20 * ch + TICKS_100MS / 2);
    l = (unsigned char) inp(port + REG_LSR);
    while ((l & LSR_DR) && *held < 64) {
        if (l & LSR_OE)
            *oe = 1;
        *last = (unsigned char) inp(port + REG_DATA);
        (*held)++;
        l = (unsigned char) inp(port + REG_LSR);
    }
    if (l & LSR_OE)
        *oe = 1;
}

/* One phase: fifo=1 wants IIR C0, 4 held, no OE. fifo=0 wants IIR 00,
 * 1 held, the last byte sent, and OE. */
static int typesw_one(const char *what, int fifo)
{
    unsigned char iir = (unsigned char) (inp(port + REG_IIR) & 0xC0), last;
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
    outp(port + REG_FCR, 0x07);
    ok &= typesw_one("16550 FIFO on", 1);
    wd_set((unsigned char) (wd_r5 & ~0x01));
    ok &= typesw_one("->16450", 0);
    outp(port + REG_FCR, 0x00);
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
    (void) inp(port + REG_MSR);
    outp(port + REG_MCR, base | up);
    t_start();
    do {
        m = (unsigned char) inp(port + REG_MSR);
        seen |= m & 0x0F;
    } while ((m & lines) != lines && t_now() < TMO_TICKS);
    t_sleep(TICKS_1MS);
    seen |= (unsigned char) (inp(port + REG_MSR) & 0x0F);
    m = (unsigned char) inp(port + REG_MSR);
    ok = (seen & deltas) == deltas && !(seen & MSR_TERI) && !(m & 0x0F);
    if (!ok)
        note("delta rise: deltas %02X, then %02X (want %02X, then 00)", seen, m & 0x0F, deltas);
    if (ri) {
        seen = 0;
        outp(port + REG_MCR, base | (up & ~ri_drv));
        t_start();
        do {
            m = (unsigned char) inp(port + REG_MSR);
            seen |= m & 0x0F;
        } while ((m & MSR_RI) && t_now() < TMO_TICKS);
        t_sleep(TICKS_1MS);
        seen |= (unsigned char) (inp(port + REG_MSR) & 0x0F);
        if (!(seen & MSR_TERI)) {
            ok = 0;
            note("delta: RI fell, deltas %02X (no TERI 04)", seen);
        }
    }
    outp(port + REG_MCR, base);
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
        outp(port + REG_FCR, 0x47);     /* FIFOs on and cleared, RX trigger 4 */
    (void) inp(port + REG_LSR);
    (void) inp(port + REG_MSR);
    (void) inp(port + REG_IIR);
    isr_hits = 0;
    isr_data = -1;
    isr_iir = 0xFF;
    isr_lsr = 0;

    old_isr = _dos_getvect(vec);
    _dos_setvect(vec, com_isr);
    mask = (unsigned char) inp(PIC1_DATA);
    outp(PIC1_DATA, mask & ~bit);
    outp(port + REG_IER, ier[kind]);

    switch (kind) {
    case IRQ_LSR:
        if (wait_thre()) {
            outp(port + REG_DATA, 0x11);
            if (wait_thre()) {
                outp(port + REG_DATA, 0x22);
                if (wait_thre())
                    outp(port + REG_DATA, 0x33);
            }
        }
        break;
    case IRQ_MSR:
        outp(port + REG_MCR, mcr ^ MCR_RTS);
        break;
    default:
        if (wait_thre())
            outp(port + REG_DATA, 0xA5);
        break;
    }
    t_start();
    while (isr_hits == 0 && t_now() < TMO_TICKS)
        ;

    outp(port + REG_IER, 0x00);
    outp(PIC1_DATA, (inp(PIC1_DATA) & ~bit) | (mask & bit));
    _dos_setvect(vec, old_isr);
    outp(port + REG_FCR, 0x00);
    outp(port + REG_MCR, mcr);

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

/* ------------------------------------------------------- latency mode */

/* SERTEST /LAT[:lines] : live full-screen view of return delay, one line
 * per pass of lat_secs and at least LAT_MIN_SAMP samples. Counts are per
 * rate. Without /B, Enter steps to the next rate and resets them. */

#define LAT_MAX      5          /* lines one panel can offer */
#define LAT_HIST     8          /* recent passes kept per line */
#define LAT_MIN_SAMP 4
#define LAT_TMO      0xFFFFFFFFUL
#define LAT_BOX      2          /* LINES box top row */
#define LAT_REC      (LAT_BOX + LAT_MAX + 3)   /* RECENT PASSES box top row */
#define LAT_STAT     (LAT_REC + LAT_MAX + 4)   /* "testing:" line */

struct lat_def {
    const char   *key;          /* /LAT: name */
    const char   *label;        /* LINES table */
    unsigned char obit, ibit;   /* 0/0 = data TX->RX */
    int           panel;        /* 1 = INT, 2 = EXT, 3 = both */
};

static const struct lat_def lat_defs[] = {
    { "DATA", "DATA TX->RX", 0,        0,       3 },
    { "RTS",  "RTS -> CTS",  MCR_RTS,  MSR_CTS, 3 },
    { "DTR",  "DTR -> DSR",  MCR_DTR,  MSR_DSR, 3 },
    { "DCD",  "DTR -> DCD",  MCR_DTR,  MSR_DCD, 2 },
    { "RI",   "DTR -> RI",   MCR_DTR,  MSR_RI,  2 },
    { "OUT1", "OUT1 -> RI",  MCR_OUT1, MSR_RI,  1 },
    { "OUT2", "OUT2 -> DCD", MCR_OUT2, MSR_DCD, 1 }
};
#define LAT_NDEFS (int) (sizeof(lat_defs) / sizeof(lat_defs[0]))

struct lat_line {
    const struct lat_def *d;
    unsigned long n, tmo, sum16, min, max, now;
    unsigned long bkt[5];
    unsigned long hist[LAT_HIST];
    int           nhist;
    int           edge;         /* next line sample: 0 rising, 1 falling */
};

/* The pass in progress for the log, reset every pass. */
static struct {
    unsigned long n, tmo, sum16, min, max;
    unsigned long bkt[5];
} lat_pass;

static struct lat_line lat[LAT_MAX];
static int nlat;
static unsigned char lat_base;  /* MCR bits held steady: MCR_LOOP or 0 */
static int lat_lap;             /* passes completed at this rate */
static int lat_paused;          /* shown as RUNNING / STOPPED by the clock */

/* Resolve lat_arg, ALL or RTS,DTR,..., against the chosen panel. Returns 0
 * on an unknown name or one the panel lacks. */
static int lat_select(void)
{
    char buf[40], *tok;
    int panel = do_ext ? 2 : 1, i, j;

    nlat = 0;
    if (strcmp(lat_arg, "ALL") == 0) {
        for (i = 0; i < LAT_NDEFS; i++)
            if ((lat_defs[i].panel & panel) && !(no_ri && lat_defs[i].ibit == MSR_RI && panel == 2))
                lat[nlat++].d = &lat_defs[i];
        return nlat > 0;
    }
    strcpy(buf, lat_arg);
    for (tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
        for (i = 0; i < LAT_NDEFS && strcmp(lat_defs[i].key, tok) != 0; i++)
            ;
        if (i == LAT_NDEFS || !(lat_defs[i].panel & panel) || nlat == LAT_MAX)
            return 0;
        for (j = 0; j < nlat; j++)
            if (lat[j].d == &lat_defs[i])
                break;
        if (j == nlat)
            lat[nlat++].d = &lat_defs[i];
    }
    return nlat > 0;
}

static void lat_reset(void)
{
    int i;
    for (i = 0; i < nlat; i++) {
        const struct lat_def *d = lat[i].d;
        memset(&lat[i], 0, sizeof(lat[i]));
        lat[i].d = d;
        lat[i].min = LAT_TMO;
    }
}

/* One sample. Data is one byte out and back. A line is one edge after the
 * input settled, settle time not counted. */
static unsigned long lat_sample(struct lat_line *l)
{
    static unsigned char pat = 0;
    unsigned long t;
    unsigned char got;
    int rising;

    if (l->d->obit == 0) {
        pat = (unsigned char) (pat + 0x35);
        return xfer_byte(pat, &t, &got) ? t : LAT_TMO;
    }
    rising = !l->edge;
    l->edge ^= 1;
    outp(port + REG_MCR, rising ? lat_base : (lat_base | l->d->obit));
    t_start();
    if (!wait_msr(l->d->ibit, rising ? 0 : l->d->ibit, &t))
        return LAT_TMO;
    t_start();
    outp(port + REG_MCR, rising ? (lat_base | l->d->obit) : lat_base);
    return wait_msr(l->d->ibit, rising ? l->d->ibit : 0, &t) ? t : LAT_TMO;
}

static void lat_record(struct lat_line *l, unsigned long t, unsigned long *worst)
{
    if (t == LAT_TMO) {
        l->tmo++;
        l->bkt[4]++;
        lat_pass.tmo++;
        lat_pass.bkt[4]++;
    } else {
        l->n++;
        l->sum16 += (t + 8) >> 4;
        if (t < l->min)
            l->min = t;
        if (t > l->max)
            l->max = t;
        l->bkt[bucket(t)]++;
        lat_pass.n++;
        lat_pass.sum16 += (t + 8) >> 4;
        if (t < lat_pass.min)
            lat_pass.min = t;
        if (t > lat_pass.max)
            lat_pass.max = t;
        lat_pass.bkt[bucket(t)]++;
    }
    if (*worst != LAT_TMO && (t == LAT_TMO || t > *worst))
        *worst = t;
}

static void lat_ms(char *buf, unsigned long t, int blank_zero)
{
    if (t == LAT_TMO)
        strcpy(buf, "TMO");
    else if (blank_zero && t == 0)
        buf[0] = 0;
    else
        fmt_ms(buf, t, 0);
}

/* ---- SERLOG.TXT: one line per completed pass. Truncated at start, every
 * write reopens, appends and closes. */

static char log_path[80];
static int  log_ok;

static void log_printf(const char *fmt, ...)
{
    FILE *f;
    va_list ap;
    if (!log_ok || (f = fopen(log_path, "a")) == NULL)
        return;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fclose(f);
}

/* Log path: argv[0]'s directory, else the current one. */
static void log_open(const char *argv0)
{
    const char *slash = argv0 ? strrchr(argv0, '\\') : NULL;
    FILE *f;
    int len = slash ? (int) (slash - argv0) + 1 : 0;

    if (len > (int) sizeof(log_path) - 13)
        len = 0;
    memcpy(log_path, argv0, len);
    strcpy(log_path + len, log_name);
    f = fopen(log_path, "w");
    log_ok = (f != NULL);
    if (f)
        fclose(f);
}

static void log_hms(char *buf)
{
    unsigned long s = run_secs();
    sprintf(buf, "%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
}

/* ms with one decimal, no "<0.1" -- the log is for machines too */
static void log_ms(char *buf, unsigned long t)
{
    unsigned long ms10 = t * 100UL / 11932UL;
    sprintf(buf, "%lu.%lu", ms10 / 10, ms10 % 10);
}

static void log_pass(const struct lat_line *l)
{
    char ts[12], mn[16], av[16], mx[16];
    log_hms(ts);
    if (lat_pass.n) {
        log_ms(mn, lat_pass.min);
        log_ms(av, lat_pass.sum16 / lat_pass.n * 16);
    } else {
        strcpy(mn, "--");
        strcpy(av, "--");
    }
    if (lat_pass.tmo)
        strcpy(mx, "TMO");
    else if (lat_pass.n)
        log_ms(mx, lat_pass.max);
    else
        strcpy(mx, "--");
    log_printf("%s rate=%lu line=%s samples=%lu tmo=%lu min=%s avg=%s max=%s"
               " <1ms=%lu <10=%lu <100=%lu <1s=%lu >1s=%lu\n",
               ts, baud_tab[cur_bi], l->d->key, lat_pass.n + lat_pass.tmo, lat_pass.tmo,
               mn, av, mx, lat_pass.bkt[0], lat_pass.bkt[1], lat_pass.bkt[2],
               lat_pass.bkt[3], lat_pass.bkt[4]);
}

static void lat_pass_reset(void)
{
    memset(&lat_pass, 0, sizeof(lat_pass));
    lat_pass.min = LAT_TMO;
}

static void lat_box(int r, int rows, const char *title)
{
    int i;
    vch(r, 0, 0xDA, A_NORM);
    vfill(r, 1, 78, 0xC4, A_NORM);
    vch(r, 79, 0xBF, A_NORM);
    vput(r, 2, A_NORM, " ");
    vput(r, 3, A_NORM, title);
    vput(r, 3 + (int) strlen(title), A_NORM, " ");
    for (i = r + 1; i <= r + rows; i++) {
        vch(i, 0, 0xB3, A_NORM);
        vfill(i, 1, 78, ' ', A_NORM);
        vch(i, 79, 0xB3, A_NORM);
    }
    vch(r + rows + 1, 0, 0xC0, A_NORM);
    vfill(r + rows + 1, 1, 78, 0xC4, A_NORM);
    vch(r + rows + 1, 79, 0xD9, A_NORM);
}

/* Rate prompt. "Enter = next rate" appears only after one full lap of the
 * lines at this rate. */
static void lat_prompt(int cycle)
{
    char buf[81];

    vfill(22, 0, 80, ' ', A_NORM);
    vfill(24, 0, 80, ' ', A_NORM);
    if (cycle) {
        if (lat_lap >= nlat)
            sprintf(buf, " rate %lu  --  Enter = next rate (%lu)", baud_tab[cur_bi],
                    baud_tab[(cur_bi + 1) % NBAUD]);
        else
            sprintf(buf, " rate %lu  --  first lap %d/%d", baud_tab[cur_bi], lat_lap, nlat);
        vput(22, 0, A_NORM, buf);
    }
    vput(24, 0, A_NORM, (cycle && lat_lap >= nlat)
         ? " Esc=quit   Space=pause   R=reset stats   N=skip to next line   Enter=next rate"
         : " Esc=quit   Space=pause   R=reset stats   N=skip to next line");
}

static void lat_frame(int cycle)
{
    static const char *hl[8] = { "-7", "-6", "-5", "-4", "-3", "-2", "-1", "now" };
    char buf[81];
    int i;

    cls();
    sprintf(buf, " XP5 SERTEST 0.1  LATENCY    COM%d %03Xh IRQ%d    UART %-7s    %s    %lu 8N1",
            com_n, port, irq, uart_name, do_ext ? "EXT" : "INT", baud_tab[cur_bi]);
    vput(0, 0, A_NORM, buf);

    strcpy(buf, " lines:");
    for (i = 0; i < nlat; i++) {
        strcat(buf, " ");
        strcat(buf, lat[i].d->key);
    }
    sprintf(buf + strlen(buf), "   %s, %ds each", nlat > 1 ? "round robin" : "single line", lat_secs);
    vput(1, 0, A_NORM, buf);

    lat_box(LAT_BOX, LAT_MAX + 1, "LINES");
    sprintf(buf, "  %-13s%8s%8s%8s%8s%6s%6s%6s%6s%6s", "line", "now", "min", "avg", "max",
            "<1ms", "<10", "<100", "<1s", ">1s");
    vput(LAT_BOX + 1, 1, A_NORM, buf);
    lat_box(LAT_REC, LAT_MAX + 1, "RECENT PASSES (worst ms per pass)");
    vput(LAT_REC + 1, 2, A_NORM, "line");
    for (i = 0; i < LAT_HIST; i++) {
        sprintf(buf, "%8s", hl[i]);
        vput(LAT_REC + 1, 15 + i * 8, A_NORM, buf);
    }

    if (log_ok)
        sprintf(buf, " log: %s  (one line per pass, overwritten each run)", log_path);
    else
        sprintf(buf, " log: %s could not be opened -- not logging", log_path);
    vput(LAT_STAT + 1, 0, A_NORM, buf);

    lat_prompt(cycle);
}

static void lat_draw(int cur)
{
    char buf[81], a[12], b[12], c[12], d[12];
    unsigned long secs;
    int i, j, k;
    struct lat_line *l;

    /* All caps and same place, so OCR can tell live from paused. */
    secs = run_secs();
    sprintf(buf, "%s %02lu:%02lu:%02lu", lat_paused ? "STOPPED" : "RUNNING",
            secs / 3600, (secs / 60) % 60, secs % 60);
    vput(1, 62, A_NORM, buf);

    for (i = 0; i < nlat; i++) {
        l = &lat[i];
        lat_ms(a, l->now, 1);
        if (l->n) {
            fmt_ms(b, l->min, 0);
            fmt_ms(c, l->sum16 / l->n * 16, 0);
        } else {
            strcpy(b, "--");
            strcpy(c, "--");
        }
        if (l->tmo)
            strcpy(d, "TMO");
        else if (l->n)
            fmt_ms(d, l->max, 0);
        else
            strcpy(d, "--");
        sprintf(buf, "%c %-13s%8s%8s%8s%8s", i == cur ? '>' : ' ', l->d->label, a, b, c, d);
        for (k = 0; k < 5; k++)
            sprintf(buf + strlen(buf), "%6lu", l->bkt[k] > 999999UL ? 999999UL : l->bkt[k]);
        vput(LAT_BOX + 2 + i, 1, (l->tmo && i == cur) ? A_BRIGHT : A_NORM, buf);

        vfill(LAT_REC + 2 + i, 1, 78, ' ', A_NORM);
        vput(LAT_REC + 2 + i, 2, A_NORM, l->d->key);
        for (j = 0; j < l->nhist; j++) {
            lat_ms(a, l->hist[j], 0);
            sprintf(buf, "%8s", a);
            vput(LAT_REC + 2 + i, 15 + (LAT_HIST - l->nhist + j) * 8,
                 l->hist[j] == LAT_TMO ? A_BRIGHT : A_NORM, buf);
        }
    }
}

static void lat_status(int cur, unsigned long n, unsigned long pass_ticks, int paused)
{
    char buf[81];
    unsigned long left10 = p_acc < pass_ticks ? (pass_ticks - p_acc) * 10UL / PIT_HZ : 0;
    vfill(LAT_STAT, 0, 80, ' ', A_NORM);
    if (paused)
        sprintf(buf, " PAUSED on %s  --  Space to resume", lat[cur].d->label);
    else if (nlat > 1)
        sprintf(buf, " testing: %-12s  %5lu samples this pass    next: %s in %lu.%lus",
                lat[cur].d->label, n, lat[(cur + 1) % nlat].d->label, left10 / 10, left10 % 10);
    else
        sprintf(buf, " testing: %-12s  %5lu samples this pass    next pass in %lu.%lus",
                lat[cur].d->label, n, left10 / 10, left10 % 10);
    vput(LAT_STAT, 0, A_NORM, buf);
}

static void run_latency(void)
{
    unsigned long pass_ticks = (unsigned long) lat_secs * PIT_HZ, n, worst, t, last_ui;
    int cur = 0, cycle = (nbaud != 1), k, paused = 0, next, i;
    char ts[12], names[40];
    struct lat_line *l;

    lat_base = do_ext ? 0x00 : MCR_LOOP;
    lat_lap = 0;
    cur_bi = cycle ? 0 : baud0;
    cur_lcr = 0x03;
    run_t0 = bios_ticks();
    uart_init(lat_base);
    lat_reset();
    lat_frame(cycle);
    lat_draw(cur);

    names[0] = 0;
    for (i = 0; i < nlat; i++) {
        if (i)
            strcat(names, ",");
        strcat(names, lat[i].d->key);
    }
    log_printf("XP5 SERTEST 0.1 latency log\n"
               "COM%d %03Xh IRQ%d UART %s %s lines=%s pass=%ds rates=%s\n"
               "# times are run time hh:mm:ss; min/avg/max in ms for that pass; bucket counts per pass\n"
               "# rate %lu\n",
               com_n, port, irq, uart_name, do_ext ? "EXT" : "INT", names, lat_secs,
               cycle ? "cycle(Enter)" : "fixed", baud_tab[cur_bi]);

    for (;;) {
        l = &lat[cur];
        n = 0;
        worst = 0;
        p_acc = 0;
        last_ui = 0;
        next = 0;
        lat_pass_reset();
        while (!next && (paused || p_acc < pass_ticks || n < LAT_MIN_SAMP)) {
            if (!paused) {
                t = lat_sample(l);
                lat_record(l, t, &worst);
                n++;
            } else
                (void) t_now();
            if (p_acc - last_ui >= PIT_HZ / 4 || p_acc < last_ui) {
                last_ui = p_acc;
                lat_status(cur, n, pass_ticks, paused);
                lat_draw(cur);
            }
            while (kbhit()) {
                k = getch();
                if (k == 0)
                    (void) getch();          /* extended key: ignore */
                else if (k == 27) {
                    outp(port + REG_MCR, lat_base);
                    log_hms(ts);
                    /* Marker the testlist polls b_drive.img for, upper-cased at run time. */
                    strcpy(names, "sertest lat done");
                    strupr(names);
                    log_printf("%s %s (Esc)\n", ts, names);
                    return;
                } else if (k == ' ') {
                    paused = !paused;
                    lat_paused = paused;
                    lat_status(cur, n, pass_ticks, paused);
                    lat_draw(cur);
                } else if (k == 'r' || k == 'R') {
                    lat_reset();
                    lat_pass_reset();
                    n = 0;
                    worst = 0;
                    lat_lap = 0;
                    lat_prompt(cycle);
                    lat_draw(cur);
                    log_hms(ts);
                    log_printf("%s # stats reset (R)\n", ts);
                } else if (k == 'n' || k == 'N') {
                    next = 1;
                } else if (k == 13 && cycle && lat_lap >= nlat) {
                    cur_bi = (cur_bi + 1) % NBAUD;
                    uart_init(lat_base);
                    lat_reset();
                    lat_lap = 0;
                    lat_frame(cycle);
                    n = 0;
                    worst = 0;
                    next = 2;               /* new rate: start the round over at line 1 */
                    log_printf("# rate %lu\n", baud_tab[cur_bi]);
                }
            }
        }
        if (n > 0 && next != 2) {
            l->now = worst;
            if (l->nhist == LAT_HIST) {
                memmove(l->hist, l->hist + 1, (LAT_HIST - 1) * sizeof(l->hist[0]));
                l->nhist--;
            }
            l->hist[l->nhist++] = worst;
            log_pass(l);                    /* between passes, never mid-sample */
            if (next == 0 && lat_lap < nlat) {
                lat_lap++;                  /* a skipped pass is not a lap step */
                lat_prompt(cycle);          /* at nlat: Enter is live */
            }
        }
        outp(port + REG_MCR, lat_base);
        if (next == 2)
            cur = 0;
        else
            cur = (cur + 1) % nlat;
        lat_draw(cur);
    }
}

/* ------------------------------------------------- page 2: baud cycle */

#define P2_SAMP   8             /* idle-line BURST samples per rate */
#define P2_SLACK  36            /* about 30 us of polling slack on every bound */
#define P2_FRAME  10            /* bits per 8N1 character */
#define P2_HDR    2             /* column header row */
#define P2_FIRST  3             /* first rate row */
#define P2_ROWS   15            /* frame content rows: header, 12 rates, 2 summary */
#define P2_GRAPH  17            /* graph offset inside the panel; 20 cells */
#define P2_SLOW_LINE (2 * TICKS_1MS)
struct p2_res {
    unsigned long bmin, bmax;           /* BURST, PIT ticks */
    unsigned long first, run, span, gmin, gmax;   /* long run timings */
    unsigned long line;                 /* worst RTS->CTS / DTR->DSR edge */
    int n;                              /* run length */
    unsigned char st;                   /* ST_PEND / ST_PASS / ST_FAIL */
    unsigned char slow;
    unsigned char cpu;                  /* RUN not run, CPU too slow */
};
static struct p2_res p2[2][NBAUD];

/* Bit period of rate i in 1/64 PIT ticks, 1.8432 MHz / 16 / divisor. */
static unsigned long bit64(int i)
{
    return ((PIT_HZ * 64UL + 57600UL) / 115200UL) * div_tab[i];
}

/* PIT ticks to tenths of a bit period at rate i */
static unsigned long bits10(unsigned long t, int i)
{
    return (t * 640UL + bit64(i) / 2) / bit64(i);
}

static void fmt_bits(char *buf, unsigned long t, int i)
{
    unsigned long b = bits10(t, i);
    if (b > 999)
        strcpy(buf, ">99");
    else
        sprintf(buf, "%lu.%lu", b / 10, b % 10);
}

/* RUN in tenths of a bit from the whole first..last DR span. Truncating
 * per byte ticks read 9.9 for 10.0. */
static unsigned long run10(const struct p2_res *q, int i)
{
    unsigned long m = (unsigned long) (q->n - 1);
    return q->n > 1 ? (bits10(q->span, i) + m / 2) / m : 0;
}

static int wait_idle(void)
{
    t_start();
    while ((inp(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT))
        if (t_now() >= TMO_TICKS)
            return 0;
    drain_rx();
    return 1;
}

/* After a timeout or wrong byte, read and discard until the line is quiet
 * for 4 character times, 100 ms at least and 3 s at most. */
static void resync(void)
{
    unsigned long quiet = 4UL * 12UL * PIT_HZ / baud_tab[cur_bi], last = 0, t;
    int n = 0;

    if (quiet < 100UL * TICKS_1MS)
        quiet = 100UL * TICKS_1MS;
    t_start();
    for (;;) {
        t = t_now();
        if (inp(port + REG_LSR) & LSR_DR) {
            (void) inp(port + REG_DATA);
            last = t;
            if (++n >= 64)
                break;
            continue;
        }
        if (t - last >= quiet || t >= 3UL * TICKS_1S)
            break;
    }
}

/* One byte on an idle line, written pre ticks after it went idle. Returns
 * ticks from the THR write to LSR.DR, 0 on timeout, -1 on a wrong byte or
 * LSR error. */
/* First wrong byte of the last xfer: sent, received, LSR and index. */
static unsigned char bad_want, bad_got, bad_lsr;
static int bad_at;

static int xfer_idle(unsigned char b, unsigned long pre, unsigned long *t)
{
    unsigned char l, c;

    if (!wait_idle())
        return 0;
    if (pre)
        t_sleep(pre);
    t_start();
    outp(port + REG_DATA, b);
    while (!((l = (unsigned char) inp(port + REG_LSR)) & LSR_DR))
        if (t_now() >= TMO_TICKS)
            return 0;
    *t = t_now();
    c = (unsigned char) inp(port + REG_DATA);
    bad_want = b; bad_got = c; bad_lsr = l; bad_at = 0;
    return (c == b && !(l & 0x0E)) ? 1 : -1;
}

/* About 1/4 s of line time, 16 bytes at least, 512 at most. */
static int run_len(unsigned long b)
{
    unsigned long n = b / 40;
    return n < 16 ? 16 : (n > 512 ? 512 : (int) n);
}

/* n bytes back to back, polled. THR is refilled when THRE sets and every
 * DR is timestamped. 0 on a 1 s stall, -1 on wrong data or LSR error, else 1. */
static int xfer_run(int n, struct p2_res *r)
{
    unsigned long t, prev = 0, gap;
    unsigned char l, c;
    int sent = 0, recv = 0, bad = 0;

    if (!wait_idle())
        return 0;
    r->gmin = LAT_TMO;
    r->gmax = 0;
    t_start();
    outp(port + REG_DATA, pat_tab[sent++ & 7]);
    for (;;) {
        l = (unsigned char) inp(port + REG_LSR);
        t = t_now();
        if (l & LSR_DR) {
            c = (unsigned char) inp(port + REG_DATA);
            if ((c != pat_tab[recv & 7] || (l & 0x0E)) && !bad) {
                bad = 1;
                bad_want = pat_tab[recv & 7]; bad_got = c; bad_lsr = l; bad_at = recv;
            }
            if (recv == 0)
                r->first = t;
            else {
                gap = t - prev;
                if (gap < r->gmin) r->gmin = gap;
                if (gap > r->gmax) r->gmax = gap;
            }
            prev = t;
            if (++recv == n)
                break;
        }
        if ((l & LSR_THRE) && sent < n)
            outp(port + REG_DATA, pat_tab[sent++ & 7]);
        if (t - (recv ? prev : 0) >= TMO_TICKS)
            return 0;
    }
    r->n = n;
    r->span = prev - r->first;
    r->run = r->span / (unsigned long) (n - 1);
    return bad ? -1 : 1;
}

/* Graph cell k covers bits k..k+1, upper half BURST worst and lower half
 * RUN. Empty cells are dots, with a bar at 10. */
static void p2_graph(int r, int c, const struct p2_res *q, int i)
{
    unsigned long b = bits10(q->bmax, i), u = run10(q, i);
    unsigned char ch;
    int k, top, bot;

    for (k = 0; k < 20; k++) {
        top = b > (unsigned long) k * 10 + 5;
        bot = u > (unsigned long) k * 10 + 5;
        ch = (unsigned char) (top && bot ? 0xDB : top ? 0xDF : bot ? 0xDC : (k == 10 ? 0xB3 : 0xFA));
        vch(r, c + k, ch, A_NORM);
    }
    if (b > 200 || u > 200)
        vch(r, c + 19, 0xAF, A_BRIGHT);       /* >> off the scale */
}

static void p2_row(int p, int i)
{
    const struct p2_res *q = &p2[p][i];
    int r = P2_FIRST + (i - baud0), c = p * 40 + 1;
    char a[12], b[12], buf[40];
    unsigned long lo, hi;

    vfill(r, c, 38, ' ', A_NORM);
    sprintf(buf, "%5s", rate_lbl[i]);
    vput(r, c, A_NORM, buf);
    if (q->st == ST_PEND)
        return;
    if (q->st == ST_FAIL && q->n == 0) {
        vput(r, c + 7, A_BRIGHT, "TIMEOUT");
        vch(r, c + 37, 'X', A_BRIGHT);
        return;
    }
    lo = bits10(q->bmin, i);
    hi = bits10(q->bmax, i);
    if (hi - lo < 5)
        fmt_bits(a, q->bmax, i);
    else if (hi > 999)
        sprintf(a, "%lu-99", (lo + 5) / 10);
    else
        sprintf(a, "%lu-%lu", (lo + 5) / 10, (hi + 5) / 10);
    lo = run10(q, i);
    if (q->cpu)
        strcpy(b, "cpu");
    else if (lo > 999)
        strcpy(b, ">99");
    else
        sprintf(b, "%lu.%lu", lo / 10, lo % 10);
    sprintf(buf, "%5s %4s", a, b);
    vput(r, c + 6, q->slow ? A_BRIGHT : A_NORM, buf);
    p2_graph(r, c + P2_GRAPH, q, i);
    vch(r, c + 37, q->st == ST_FAIL ? 'X' : q->slow ? '*' : ' ', A_BRIGHT);
}

static int p2_tally(int p, int *total, int *slow)
{
    int i, pass = 0;
    *total = *slow = 0;
    for (i = baud0; i < baud0 + nbaud; i++) {
        if (p2[p][i].st == ST_PEND)
            continue;
        (*total)++;
        if (p2[p][i].st == ST_PASS)
            pass++;
        *slow += p2[p][i].slow;
    }
    return pass;
}

static void p2_summary(int p)
{
    char buf[40], ms[16];
    unsigned long w = 0;
    int i, wi = -1, pass, total, slow, c = p * 40 + 2;

    for (i = baud0; i < baud0 + nbaud; i++)
        if (p2[p][i].st != ST_PEND && (wi < 0 || p2[p][i].line > w)) {
            w = p2[p][i].line;
            wi = i;
        }
    vfill(P2_FIRST + NBAUD, c, 37, ' ', A_NORM);
    vfill(P2_FIRST + NBAUD + 1, c, 37, ' ', A_NORM);
    if (wi < 0)
        return;
    fmt_ms(ms, w, w >= TMO_TICKS);
    sprintf(buf, "RTS/DTR follow  worst %s @%s", ms, rate_lbl[wi]);
    vput(P2_FIRST + NBAUD, c, w > P2_SLOW_LINE ? A_BRIGHT : A_NORM, buf);
    pass = p2_tally(p, &total, &slow);
    sprintf(buf, "rates ok %d/%d  slow %d", pass, total, slow);
    vput(P2_FIRST + NBAUD + 1, c, (pass != total) ? A_BRIGHT : A_NORM, buf);
}

static void draw_page2(int fresh)
{
    char buf[48];
    int p, i, on[2];

    on[0] = do_int;
    on[1] = do_ext && !ext_noplug;
    cur_page = 2;
    draw_header(2);
    for (p = 0; p < 2; p++) {
        draw_frame(p * 40, 1, P2_ROWS, panel_title[p]);
        if (!on[p]) {
            vput(P2_FIRST, p * 40 + 3, A_NORM, p && do_ext ? "skipped: no plug" : "not selected");
            continue;
        }
        sprintf(buf, " RATE BURST  RUN");
        vput(P2_HDR, p * 40 + 1, A_NORM, buf);
        vput(P2_HDR, p * 40 + 1 + P2_GRAPH, A_NORM, "0");
        vput(P2_HDR, p * 40 + 1 + P2_GRAPH + 5, A_NORM, "5");
        vput(P2_HDR, p * 40 + 1 + P2_GRAPH + 10, A_NORM, "10");
        vput(P2_HDR, p * 40 + 1 + P2_GRAPH + 15, A_NORM, "15");
        vput(P2_HDR, p * 40 + 1 + P2_GRAPH + 18, A_NORM, "20");
        for (i = baud0; i < baud0 + nbaud; i++) {
            if (fresh)
                memset(&p2[p][i], 0, sizeof(p2[p][i]));
            p2_row(p, i);
        }
        p2_summary(p);
    }
}

/* One rate on one panel. */
static void p2_rate(int p, int i)
{
    struct p2_res *q = &p2[p][i];
    unsigned char mcr = p ? 0x00 : MCR_LOOP;
    unsigned long t, frame, line = 0;
    int s, rc, err = 0;

    cur_bi = i;
    cur_lcr = 0x03;
    uart_init(mcr);
    t_sleep(RATE_SETTLE);
    frame = P2_FRAME * bit64(i) / 64;
    q->bmin = LAT_TMO;
    q->bmax = 0;
    for (s = 0; s < P2_SAMP; s++) {
        key_poll();
        if (stop_req)
            return;
        rc = xfer_idle(pat_tab[s], frame * s / P2_SAMP, &t);
        if (rc == 0) {
            q->st = ST_FAIL;
            note("%lu: idle byte timed out", baud_tab[i]);
            resync();
            return;
        }
        if (rc < 0 && !err) {
            err = 1;
            note("%lu: idle byte %02X came back wrong: got %02X lsr %02X", baud_tab[i], bad_want, bad_got, bad_lsr);
        }
        if (rc < 0)
            resync();
        if (t < q->bmin) q->bmin = t;
        if (t > q->bmax) q->bmax = t;
    }
    key_poll();
    if (stop_req)
        return;
    q->cpu = (unsigned char) (i > cpu_top);
    if (q->cpu) {
        q->n = 0;
        q->first = q->run = q->span = q->gmin = q->gmax = 0;
    }
    rc = q->cpu ? 1 : xfer_run(run_len(baud_tab[i]), q);
    if (rc == 0) {
        q->st = ST_FAIL;
        q->n = 0;
        note("%lu: long run stalled", baud_tab[i]);
        resync();
        return;
    }
    if (rc < 0) {
        err = 1;
        note("%lu: long run data or LSR error: #%d sent %02X got %02X lsr %02X", baud_tab[i], bad_at, bad_want, bad_got, bad_lsr);
        resync();
    }
    key_poll();
    if (stop_req)
        return;
    if (!line_test(mcr, MCR_RTS, MSR_CTS, 2, &line)) {
        err = 1;
        note("%lu: CTS did not follow RTS", baud_tab[i]);
    }
    if (!line_test(mcr, MCR_DTR, MSR_DSR, 2, &line)) {
        err = 1;
        note("%lu: DSR did not follow DTR", baud_tab[i]);
    }
    q->line = line;
    q->st = (unsigned char) (err ? ST_FAIL : ST_PASS);
    {
        unsigned long bound = (12UL * bit64(i)) / 64 + P2_SLACK;

        q->slow = (unsigned char) (q->bmax > bound
                                   || run10(q, i) > P2_FRAME * 10 + 3 + bits10(P2_SLACK, i)
                                   || line > P2_SLOW_LINE);
    }
}

static void run_page2(void)
{
    char name[32];
    int p, i, on[2], total, done = 0;

    on[0] = do_int;
    on[1] = do_ext && !ext_noplug;
    total = (on[0] + on[1]) * nbaud;
    for (p = 0; p < 2; p++) {
        if (!on[p])
            continue;
        note_panel = p ? "EXT" : "INT";
        for (i = baud0; i < baud0 + nbaud; i++) {
            key_poll();
            if (stop_req)
                break;
            sprintf(name, "%s %luB RATE", note_panel, baud_tab[i]);
            stage(name, "TESTING");
            progress(done, total);
            p2_rate(p, i);
            if (stop_req)
                break;
            p2_row(p, i);
            p2_summary(p);
            stage(name, p2[p][i].st == ST_PASS ? "PASS" : "FAIL");
            done++;
        }
        if (stop_req)
            break;
    }
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(0x00);
}

/* -------------------------------------------- pages 3/4: latency, ms */

#define P3_SAMP   16            /* round trips per rate, edges per line */
#define P3_NLINE  4
#define P3_HDR    2             /* rate column header */
#define P3_FIRST  3             /* first rate row */
#define P3_RULE   (P3_FIRST + NBAUD)    /* rule above the lines */
#define P3_LHDR   (P3_RULE + 1)         /* line column header */
#define P3_LFIRST (P3_LHDR + 1)         /* first line row */
#define P3_BOT    (P3_LFIRST + P3_NLINE)
#define P3_SPLIT  36            /* divider between numbers and graph */
#define P3_GX     37            /* graph: 36 cells, 9 per decade, 0.1 ms to 1 s */
#define P3_GW     36
#define P3_STX    74            /* STAT column */

struct p3_res {
    unsigned long min, max, sum;        /* PIT ticks */
    unsigned long v[P3_SAMP];           /* every sample, for the deviation */
    int n, tmo, bad;
    unsigned char st, slow;
    int ns;                             /* data rows: samples in send order */
    unsigned long seq[P3_SAMP];         /* PIT ticks, LAT_TMO for a timeout */
    unsigned char sent[P3_SAMP], ok[P3_SAMP];
    unsigned long warm;                 /* data rows: warm-up round trip, LAT_TMO if none */
    unsigned char warm_ok;
    unsigned long zmin, zmax, zsum;     /* data rows: the 0x00 round trips */
    int zn;
    unsigned char hold;                 /* every 0x00 held over P3_HOLD */
};
static struct p3_res p3[2][NBAUD];      /* [0 INT, 1 EXT][rate]: data round trip */
static struct p3_res p3l[2][P3_NLINE];  /* [panel][line] at the top rate */

/* Line k on each panel: MCR bit driven, MSR bit that follows it. */
static const char *p3_lname[2][P3_NLINE][2] = {
    { { "RTS", "CTS" }, { "DTR", "DSR" }, { "OUT2", "DCD" }, { "OUT1", "RI" } },
    { { "RTS", "CTS" }, { "DTR", "DSR" }, { "DTR", "DCD" },  { "DTR", "RI" } }
};
static const unsigned char p3_lout[2][P3_NLINE] = {
    { MCR_RTS, MCR_DTR, MCR_OUT2, MCR_OUT1 }, { MCR_RTS, MCR_DTR, MCR_DTR, MCR_DTR }
};
static const unsigned char p3_lin[P3_NLINE] = { MSR_CTS, MSR_DSR, MSR_DCD, MSR_RI };

/* Graph scale in PIT ticks, 0.1 ms times 10 to the j/18, j = 0..72. Cell k
 * spans entries 2k..2k+2. */
static const unsigned long p3_scale[73] = {
    119UL, 136UL, 154UL, 175UL, 199UL, 226UL, 257UL, 292UL, 332UL, 377UL, 429UL, 487UL,
    554UL, 629UL, 715UL, 813UL, 924UL, 1050UL, 1193UL, 1356UL, 1541UL, 1751UL, 1990UL,
    2262UL, 2571UL, 2921UL, 3320UL, 3773UL, 4288UL, 4873UL, 5538UL, 6294UL, 7153UL,
    8129UL, 9238UL, 10499UL, 11932UL, 13560UL, 15411UL, 17514UL, 19903UL, 22620UL,
    25706UL, 29214UL, 33201UL, 37732UL, 42881UL, 48732UL, 55383UL, 62940UL, 71529UL,
    81291UL, 92384UL, 104991UL, 119318UL, 135601UL, 154105UL, 175135UL, 199035UL,
    226196UL, 257063UL, 292143UL, 332010UL, 377317UL, 428807UL, 487324UL, 553826UL,
    629403UL, 715294UL, 812905UL, 923837UL, 1049908UL, 1193182UL
};

static int p3_on(int p)
{
    return p ? (do_ext && !ext_noplug) : do_int;
}

/* ms for the tables: two decimals under 1 ms, one under 100, none above. */
static void p3_ms(char *buf, unsigned long t)
{
    unsigned long u = t * 1000UL / 11932UL;    /* 1/100 ms; t stays under ~1.3 s */
    if (u < 100)
        sprintf(buf, "%lu.%02lu", u / 100, u % 100);
    else if (u < 10000)
        sprintf(buf, "%lu.%lu", u / 100, (u / 10) % 10);
    else
        sprintf(buf, "%lu", u / 100);
}

static void p3_add(struct p3_res *q, unsigned long t)
{
    if (t == LAT_TMO) {
        q->tmo++;
        return;
    }
    if (q->n == 0 || t < q->min)
        q->min = t;
    if (t > q->max)
        q->max = t;
    q->sum += t;
    if (q->n < P3_SAMP)
        q->v[q->n] = t;
    q->n++;
}

/* A 0x00 round trip with its own min, max and sum, so a host hold stays
 * out of the link numbers. */
static void p3_zadd(struct p3_res *q, unsigned long t)
{
    if (t == LAT_TMO) {
        q->tmo++;
        return;
    }
    if (q->zn == 0 || t < q->zmin)
        q->zmin = t;
    if (t > q->zmax)
        q->zmax = t;
    q->zsum += t;
    q->zn++;
}

/* Standard deviation in PIT ticks, integer only. Deviations are shifted to
 * fit 16 bits and each square is divided by n before summing. */
static unsigned long p3_sigma(const struct p3_res *q)
{
    unsigned long mean, d, dmax = 0, ms = 0, r = 0, bit;
    int i, n = q->n < P3_SAMP ? q->n : P3_SAMP, s = 0;

    if (n < 2)
        return 0;
    mean = q->sum / (unsigned long) q->n;
    for (i = 0; i < n; i++) {
        d = q->v[i] > mean ? q->v[i] - mean : mean - q->v[i];
        if (d > dmax)
            dmax = d;
    }
    while ((dmax >> s) > 65535UL)
        s++;
    for (i = 0; i < n; i++) {
        d = q->v[i] > mean ? q->v[i] - mean : mean - q->v[i];
        d >>= s;
        ms += d * d / (unsigned long) n;
    }
    for (bit = 1UL << 30; bit > ms; bit >>= 2)
        ;
    for (; bit; bit >>= 2) {
        if (ms >= r + bit) {
            ms -= r + bit;
            r = (r >> 1) + bit;
        } else
            r >>= 1;
    }
    return r << s;
}

/* Graph cell of t, and whether t is in its lower half. */
static int p3_cell(unsigned long t, int *lower)
{
    int j = 0;
    while (j < 72 && t >= p3_scale[j + 1])
        j++;
    *lower = !(j & 1);
    return j / 2 < P3_GW ? j / 2 : P3_GW - 1;
}

/* Solid bar from 0.1 ms to AVG, shading on to MAX. Rules at 1, 10 and
 * 100 ms. A timeout runs the bar off the end. */
static void p3_graph(int r, const struct p3_res *q)
{
    int k, a, m, lower, dummy;

    for (k = 0; k < P3_GW; k++)
        vch(r, P3_GX + k, (k % 9 == 0 && k) ? 0xB3 : 0xFA, A_NORM);
    if (q->st == ST_PEND || q->st == ST_NC)
        return;
    if (q->tmo || !q->n) {
        vfill(r, P3_GX, P3_GW - 1, 0xDB, A_NORM);
        vch(r, P3_GX + P3_GW - 1, 0xAF, A_BRIGHT);
        return;
    }
    a = p3_cell(q->sum / (unsigned long) q->n, &lower);
    m = p3_cell(q->max, &dummy);
    for (k = 0; k < a; k++)
        vch(r, P3_GX + k, 0xDB, A_NORM);
    vch(r, P3_GX + a, lower ? 0xDD : 0xDB, A_NORM);
    for (k = a + 1; k <= m; k++)
        vch(r, P3_GX + k, 0xB0, A_NORM);
    if (q->hold)
        vch(r, P3_GX + p3_cell(q->zsum / (unsigned long) q->zn, &dummy), '0', A_BRIGHT);
}

/* MIN AVG MAX JITTER after a label already in buf, then graph and STAT. */
static void p3_cols(int r, char *buf, const struct p3_res *q, int w, int dev)
{
    static const char *word[] = { "", "", "ok", "FAIL", "", "n/c", "" };
    char a[12], b[12], c[12], d[12];
    const char *st;
    int hot = q->st == ST_FAIL || q->slow;

    if (q->st == ST_PEND || q->st == ST_NC || !q->n) {
        strcpy(a, q->st == ST_FAIL ? "--" : "");
        strcpy(b, a);
        strcpy(c, q->tmo ? "TMO" : a);
        strcpy(d, a);
    } else {
        p3_ms(a, q->min);
        p3_ms(b, q->sum / (unsigned long) q->n);
        if (q->tmo)
            strcpy(c, "TMO");
        else
            p3_ms(c, q->max);
        p3_ms(d, dev ? p3_sigma(q) : q->max - q->min);
    }
    sprintf(buf + strlen(buf), "%*s%7s%7s%7s", w, a, b, c, d);
    vfill(r, 1, P3_SPLIT - 1, ' ', A_NORM);
    vput(r, 1, hot ? A_BRIGHT : A_NORM, buf);
    p3_graph(r, q);
    st = q->tmo ? "TMO" : q->bad ? "FAIL" : (q->st == ST_PASS && q->slow) ? "SLOW" : (q->st == ST_PASS && q->hold) ? "HOLD"
       : word[q->st];
    vfill(r, P3_STX, 5, ' ', A_NORM);
    vput(r, P3_STX, hot ? A_BRIGHT : A_NORM, st);
}

static void p3_row(int p, int i)
{
    char buf[48];
    sprintf(buf, " %6lu", baud_tab[i]);
    p3_cols(P3_FIRST + (i - baud0), buf, &p3[p][i], 7, 0);
}

static void p3_lrow(int p, int k)
{
    char buf[48];
    sprintf(buf, " %s\x1A%s", p3_lname[p][k][0], p3_lname[p][k][1]);
    sprintf(buf + strlen(buf), "%*s", 9 - (int) strlen(buf), "");
    p3_cols(P3_LFIRST + k, buf, &p3l[p][k], 5, 1);
}

/* Column headers: "RATE" or "LINE" and the graph's decade labels. */
static void p3_head(int r, int lines)
{
    char buf[48];
    if (lines)
        sprintf(buf, " %-8s%5s%7s%7s%7s", "LINE", "MIN", "AVG", "MAX", "DEV");
    else
        sprintf(buf, " %6s%7s%7s%7s%7s", "RATE", "MIN", "AVG", "MAX", "JITTER");
    vput(r, 1, A_NORM, buf);
    vput(r, P3_GX, A_NORM, "0.1");
    vput(r, P3_GX + 9, A_NORM, "1");
    vput(r, P3_GX + 18, A_NORM, "10");
    vput(r, P3_GX + 27, A_NORM, "100");
    vput(r, P3_GX + 34, A_NORM, "1s");
    vput(r, P3_STX, A_NORM, "STAT");
}

/* Rates on panel p whose 0x00s were held, and the least and most the 0x00
 * AVG ran over the link AVG. */
static int p3_holds(int p, unsigned long *lo, unsigned long *hi)
{
    const struct p3_res *q;
    unsigned long d;
    int i, n = 0;

    *lo = *hi = 0;
    for (i = baud0; i < baud0 + nbaud; i++) {
        q = &p3[p][i];
        if (!q->hold)
            continue;
        d = q->zsum / (unsigned long) q->zn - q->sum / (unsigned long) q->n;
        if (!n || d < *lo)
            *lo = d;
        if (d > *hi)
            *hi = d;
        n++;
    }
    return n;
}

/* The legend under page 3/4: how much the host held 0x00 back, if it did. */
static void p3_legend(int p)
{
    char buf[96], a[12], b[12];
    unsigned long lo, hi;
    int n = p3_holds(p, &lo, &hi);

    vfill(LEG_ROW, 1, 78, ' ', A_NORM);
    if (!n) {
        vput(LEG_ROW, 1, A_NORM, "RTT: THR write to LSR.DR, 8N1. SLOW: >12 bits, >2 ms ");
        return;
    }
    p3_ms(a, lo);
    p3_ms(b, hi);
    if (lo == hi || !strcmp(a, b))
        sprintf(buf, "HOLD: 0x00 +%s ms over link MAX at %d rate(s). '0' = 0x00 AVG", a, n);
    else
        sprintf(buf, "HOLD: 0x00 +%s-%s ms over link MAX at %d rate(s). '0' = 0x00 AVG", a, b, n);
    vput(LEG_ROW, 1, A_BRIGHT, buf);
}

/* Page 3 is the EXTERNAL panel, page 4 the INTERNAL one. */
static void draw_page3(int p, int fresh)
{
    char buf[64];
    int i, k;

    cur_page = p ? 3 : 4;
    draw_header(cur_page);
    sprintf(buf, "%s \xC4\xC4 link latency & jitter, ms, bytes 01-FF", p ? "EXTERNAL" : "INTERNAL");
    box(1, P3_BOT, buf);
    sprintf(buf, "MODEM LINES at %lu, %d edges", baud_tab[top_bi()], P3_SAMP);
    box_rule(P3_RULE, buf);
    box_split(P3_SPLIT, P3_HDR, P3_RULE - 1, 0xC2, 0xC5);
    box_split(P3_SPLIT, P3_LHDR, P3_BOT - 1, 0xC5, 0xC1);
    if (!p3_on(p)) {
        vput(P3_FIRST, 2, A_NORM, p && do_ext ? "skipped: no plug" : "not selected");
        return;
    }
    p3_head(P3_HDR, 0);
    p3_head(P3_LHDR, 1);
    for (i = baud0; i < baud0 + nbaud; i++) {
        if (fresh)
            memset(&p3[p][i], 0, sizeof(p3[p][i]));
        p3_row(p, i);
    }
    for (k = 0; k < P3_NLINE; k++) {
        if (fresh) {
            memset(&p3l[p][k], 0, sizeof(p3l[p][k]));
            if (p && k == 3 && no_ri)
                p3l[p][k].st = ST_NC;
        }
        p3_lrow(p, k);
    }
    p3_legend(p);
}

/* PIT ticks over which a data round trip at rate i is SLOW. */
static unsigned long p3_bound(int i)
{
    return (12UL * bit64(i)) / 64 + P2_SLACK;
}

#define P3_WARM   0x55          /* warm-up byte, sent once after each rate change */
#define P3_HOLD   (2 * TICKS_1MS)       /* 0x00 over the link's MAX by more: held */

/* One warm-up round trip after the rate change, not counted, then
 * RATE_SETTLE, then 16 round trips, the 8 patterns twice. A wrong byte or
 * timeout fails the rate and two timeouts end it. */
static void p3_rate(int p, int i)
{
    struct p3_res *q = &p3[p][i];
    unsigned long t;
    unsigned char got, b, fw = 0, fg = 0, fl = 0;
    int s, fs = -1;

    memset(q, 0, sizeof(*q));
    cur_bi = i;
    cur_lcr = 0x03;
    uart_init(p ? 0x00 : MCR_LOOP);
    q->warm = LAT_TMO;
    if (xfer_byte(P3_WARM, &t, &got)) {
        q->warm = t;
        q->warm_ok = (unsigned char) (got == P3_WARM && !(last_lsr & 0x0E));
    }
    if (q->warm == LAT_TMO || !q->warm_ok)
        resync();
    t_sleep(RATE_SETTLE);
    for (s = 0; s < P3_SAMP && q->tmo < 2; s++) {
        key_poll();
        if (stop_req)
            return;
        progress(s, P3_SAMP);
        b = pat_tab[s & 7];
        q->sent[q->ns] = b;
        if (!xfer_byte(b, &t, &got)) {
            q->seq[q->ns] = LAT_TMO;
            q->ok[q->ns++] = 0;
            p3_add(q, LAT_TMO);
            resync();
            continue;
        }
        q->seq[q->ns] = t;
        q->ok[q->ns++] = (unsigned char) (got == b && !(last_lsr & 0x0E));
        if (b == 0x00)
            p3_zadd(q, t);
        else
            p3_add(q, t);
        if (got != b || (last_lsr & 0x0E)) {
            if (!q->bad) { fs = s; fw = b; fg = got; fl = last_lsr; }
            q->bad++;
            resync();
        }
    }
    progress(P3_SAMP, P3_SAMP);
    q->st = (unsigned char) ((q->tmo || q->bad || !q->n) ? ST_FAIL : ST_PASS);
    q->slow = (unsigned char) (q->n && q->max > p3_bound(i));
    q->hold = (unsigned char) (q->n && q->zn && q->zmin > q->max + P3_HOLD);
    if (q->tmo)
        note("lat %lu: %d round trip(s) timed out", baud_tab[i], q->tmo);
    if (q->bad)
        note("lat %lu: %d byte(s) back wrong; first #%d sent %02X got %02X lsr %02X", baud_tab[i], q->bad, fs, fw, fg, fl);
}

/* 16 edges on line k, rising and falling in turn, timed from the MCR
 * write to the MSR bit. */
static void p3_line(int p, int k)
{
    struct p3_res *q = &p3l[p][k];
    unsigned char base = p ? 0x00 : MCR_LOOP, ob = p3_lout[p][k], ib = p3_lin[k];
    unsigned long t;
    int s, rising;

    memset(q, 0, sizeof(*q));
    if (p && k == 3 && no_ri) {
        q->st = ST_NC;
        return;
    }
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(base);
    for (s = 0; s < P3_SAMP && q->tmo < 2; s++) {
        key_poll();
        if (stop_req)
            break;
        rising = !(s & 1);
        outp(port + REG_MCR, rising ? base : (base | ob));
        t_start();
        if (!wait_msr(ib, rising ? 0 : ib, &t)) {
            q->seq[q->ns] = LAT_TMO;
            q->sent[q->ns++] = (unsigned char) rising;
            p3_add(q, LAT_TMO);
            continue;
        }
        t_start();
        outp(port + REG_MCR, rising ? (base | ob) : base);
        if (!wait_msr(ib, rising ? ib : 0, &t))
            t = LAT_TMO;
        q->seq[q->ns] = t;
        q->ok[q->ns] = 1;
        q->sent[q->ns++] = (unsigned char) rising;
        p3_add(q, t);
    }
    outp(port + REG_MCR, base);
    q->st = (unsigned char) ((q->tmo || !q->n) ? ST_FAIL : ST_PASS);
    q->slow = (unsigned char) (q->n && q->max > P2_SLOW_LINE);
    if (q->tmo)
        note("lat %s->%s: %d edge(s) timed out", p3_lname[p][k][0], p3_lname[p][k][1], q->tmo);
}

/* Pages 5/6: pages 3/4's samples one by one, same rows. */
#define P5_WX     8             /* WARM cell */
#define P5_SPLIT  12            /* divider between WARM and the samples */
#define P5_SX     13            /* first sample cell; 4 columns per cell */

/* A sample in 3 columns: .13  1.6  42  471, 1s+ past a second. */
static void p5_ms(char *buf, unsigned long t)
{
    unsigned long u;

    if (t == LAT_TMO) {
        strcpy(buf, "TMO");
        return;
    }
    u = t * 1000UL / 11932UL;           /* 1/100 ms; t stays under ~1.3 s */
    if (u < 100)
        sprintf(buf, ".%02lu", u);
    else if (u < 1000)
        sprintf(buf, "%lu.%lu", u / 100, (u / 10) % 10);
    else if (u < 100000UL)
        sprintf(buf, "%lu", u / 100);
    else
        strcpy(buf, "1s+");
}

/* One cell at column c; bright when over bound, wrong, or timed out. */
static void p5_cell(int r, int c, unsigned long t, int ok, unsigned long bound)
{
    char a[8], b[8];

    if (t != LAT_TMO && !ok)
        strcpy(a, "BAD");
    else
        p5_ms(a, t);
    sprintf(b, "%4s", a);
    vput(r, c, (t == LAT_TMO || !ok || t > bound) ? A_BRIGHT : A_NORM, b);
}

static void p5_row(int r, const struct p3_res *q, unsigned long bound, int warm)
{
    int k;

    if (q->st == ST_NC) {
        vput(r, P5_SX + 1, A_NORM, "n/c");
        return;
    }
    if (warm && q->ns)
        p5_cell(r, P5_WX, q->warm, q->warm_ok, bound);
    for (k = 0; k < q->ns; k++)
        p5_cell(r, P5_SX + 4 * k, q->seq[k], q->ok[k], bound);
}

/* Page 5 is the EXTERNAL panel, page 6 the INTERNAL one. */
static void draw_page5(int p)
{
    char buf[48];
    int i, k;

    cur_page = p ? 5 : 6;
    draw_header(cur_page);
    sprintf(buf, "%s \xC4\xC4 samples in send order, ms", p ? "EXTERNAL" : "INTERNAL");
    box(1, P3_BOT, buf);
    sprintf(buf, "MODEM LINES at %lu, %d edges", baud_tab[top_bi()], P3_SAMP);
    box_rule(P3_RULE, buf);
    box_split(P5_SPLIT, P3_HDR, P3_RULE - 1, 0xC2, 0xC5);
    box_split(P5_SPLIT, P3_LHDR, P3_BOT - 1, 0xC5, 0xC1);
    vput(LEG_ROW, 1, A_NORM, "WARM: 1st round trip, not counted. 00: not in page 3/4 MIN/AVG/MAX.");
    if (!p3_on(p)) {
        vput(P3_FIRST, 2, A_NORM, p && do_ext ? "skipped: no plug" : "not selected");
        return;
    }
    vput(P3_HDR, 1, A_NORM, "   RATE");
    vput(P3_HDR, P5_WX, A_NORM, "WARM");
    vput(P3_LHDR, 1, A_NORM, " LINE");
    for (k = 0; k < P3_SAMP; k++) {
        sprintf(buf, "  %02X", pat_tab[k & 7]);
        vput(P3_HDR, P5_SX + 4 * k, A_NORM, buf);
        vput(P3_LHDR, P5_SX + 4 * k, A_NORM, (k & 1) ? "  dn" : "  up");
    }
    for (i = baud0; i < baud0 + nbaud; i++) {
        sprintf(buf, " %6lu", baud_tab[i]);
        vput(P3_FIRST + (i - baud0), 1, A_NORM, buf);
        p5_row(P3_FIRST + (i - baud0), &p3[p][i], p3_bound(i), 1);
    }
    for (k = 0; k < P3_NLINE; k++) {
        sprintf(buf, " %s\x1A%s", p3_lname[p][k][0], p3_lname[p][k][1]);
        vput(P3_LFIRST + k, 1, A_NORM, buf);
        p5_row(P3_LFIRST + k, &p3l[p][k], P2_SLOW_LINE, 0);
    }
}

static void run_page3(int p)
{
    char name[32], a[12], b[12];
    unsigned long lo, hi;
    int i, k;

    note_panel = p ? "EXT" : "INT";
    for (i = baud0; i < baud0 + nbaud; i++) {
        key_poll();
        if (stop_req)
            break;
        sprintf(name, "%s %luB LAT", note_panel, baud_tab[i]);
        stage(name, "TESTING");
        p3_rate(p, i);
        if (stop_req)
            break;
        p3_row(p, i);
        stage(name, p3[p][i].st == ST_PASS ? "PASS" : "FAIL");
    }
    p3_legend(p);
    if (p3_holds(p, &lo, &hi)) {
        p3_ms(a, lo);
        p3_ms(b, hi);
        note("lat: 0x00 +%s..%s ms over link MAX", a, b);
    }
    for (k = 0; k < P3_NLINE && !stop_req; k++) {
        if (p && k == 3 && no_ri)
            continue;
        sprintf(name, "%s %s-%s LAT", note_panel, p3_lname[p][k][0], p3_lname[p][k][1]);
        stage(name, "TESTING");
        p3_line(p, k);
        if (stop_req)
            break;
        p3_lrow(p, k);
        stage(name, p3l[p][k].st == ST_PASS ? "PASS" : "FAIL");
    }
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(0x00);
}

/* Rows run on pages 3/4 that passed, out of those run; slow ones counted. */
static int p3_tally(int *total, int *slow)
{
    int p, i, pass = 0;
    *total = *slow = 0;
    for (p = 0; p < 2; p++) {
        if (!p3_on(p))
            continue;
        for (i = baud0; i < baud0 + nbaud + P3_NLINE; i++) {
            const struct p3_res *q = i < baud0 + nbaud ? &p3[p][i] : &p3l[p][i - baud0 - nbaud];
            if (q->st != ST_PASS && q->st != ST_FAIL)
                continue;
            (*total)++;
            pass += q->st == ST_PASS;
            *slow += q->slow;
        }
    }
    return pass;
}

/* -------------------------------------------------------- log + result */

/* Page 1 checks run on panel p that passed, out of those run. */
static int tally(int p, int *total)
{
    int i, pass = 0, st;
    *total = 0;
    for (i = 0; i < NCHK; i++) {
        st = chk_st[p][i];
        if (st != ST_PASS && st != ST_FAIL)
            continue;
        (*total)++;
        if (st == ST_PASS)
            pass++;
    }
    return pass;
}

static void log_p3(int p, const char *what, const struct p3_res *q)
{
    char a[12], b[12], c[12], d[12], e[12], f[12], g[12], h[12];
    unsigned long zavg, lavg, amin, amax;
    if (q->st != ST_PASS && q->st != ST_FAIL)
        return;
    if (q->n) {
        p3_ms(a, q->min);
        p3_ms(b, q->sum / (unsigned long) q->n);
        p3_ms(c, q->max);
        p3_ms(d, q->max - q->min);
        p3_ms(e, p3_sigma(q));
    } else
        strcpy(a, strcpy(b, strcpy(c, strcpy(d, strcpy(e, "--")))));
    log_printf("P3 %s %s ms min=%s avg=%s max=%s jitter=%s dev=%s n=%d tmo=%d bad=%d | %s%s\n",
               p ? "EXT" : "INT", what, a, b, c, d, e, q->n, q->tmo, q->bad,
               q->st == ST_PASS ? "ok" : "FAIL", q->slow ? " SLOW" : "");
    if (!q->zn)
        return;
    /* The 0x00s apart, their AVG offset from the link's, and every byte together. */
    zavg = q->zsum / (unsigned long) q->zn;
    lavg = q->n ? q->sum / (unsigned long) q->n : zavg;
    amin = (q->n && q->min < q->zmin) ? q->min : q->zmin;
    amax = q->max > q->zmax ? q->max : q->zmax;
    p3_ms(a, q->zmin);
    p3_ms(b, zavg);
    p3_ms(c, q->zmax);
    d[0] = (char) (zavg >= lavg ? '+' : '-');
    p3_ms(d + 1, zavg >= lavg ? zavg - lavg : lavg - zavg);
    p3_ms(e, amin);
    p3_ms(f, (q->sum + q->zsum) / (unsigned long) (q->n + q->zn));
    p3_ms(g, amax);
    p3_ms(h, amax - amin);
    log_printf("P3 %s %s 0x00 ms min=%s avg=%s max=%s n=%d over-link=%s%s | all bytes min=%s avg=%s max=%s jitter=%s\n",
               p ? "EXT" : "INT", what, a, b, c, q->zn, d, q->hold ? " HOLD" : "", e, f, g, h);
}

/* Every sample in send order. Data rows as sent-byte:ms, line rows as
 * up:/dn:ms, "tmo" for a timeout and "!" after a wrong byte. */
static void log_s3(int p, const char *what, const struct p3_res *q, int lines)
{
    char line[16 * P3_SAMP + 64], ms[12];
    int k, n;

    if (!q->ns)
        return;
    n = sprintf(line, "S3 %s %s ms", p ? "EXT" : "INT", what);
    if (!lines) {
        if (q->warm == LAT_TMO)
            strcpy(ms, "tmo");
        else
            p3_ms(ms, q->warm);
        n += sprintf(line + n, " warm %02X:%s%s |", P3_WARM, ms, q->warm_ok || q->warm == LAT_TMO ? "" : "!");
    }
    for (k = 0; k < q->ns; k++) {
        if (q->seq[k] == LAT_TMO)
            strcpy(ms, "tmo");
        else
            p3_ms(ms, q->seq[k]);
        if (lines)
            n += sprintf(line + n, " %s:%s", q->sent[k] ? "up" : "dn", ms);
        else
            n += sprintf(line + n, " %02X:%s%s", q->sent[k], ms, q->ok[k] || q->seq[k] == LAT_TMO ? "" : "!");
    }
    log_printf("%s\n", line);
}

/* Full-test SERLOG.TXT, written once everything has run. The last line is
 * the testlists' finished marker, upper-cased at run time. perf_ab.py reads
 * the P2 lines and the INT= EXT= RATES= SLOW= fields of the DONE line, so
 * keep their shape. */
static void log_full(int pass, const char *summary)
{
    static const char *st[] = { "PEND", "RUN", "PASS", "FAIL", "SKIP", "n/c", "n/a" };
    char done[16], a[6][12], m[4][12], ln[12], what[24];
    struct p2_res *q;
    int i, j, k;

    log_printf("XP5 SERTEST 0.1 full test\n"
               "COM%d %03Xh IRQ%d UART %s page1-rate=%lu page2-rates=%s\n",
               com_n, port, irq, uart_name, baud_tab[top_bi()],
               nbaud == 1 ? rate_lbl[baud0] : "110-115200");
    log_printf("HW CPU %s poll=%lu.%luus run-max=%lu fifo=%s scr=%s burst-rate=%s typesw=%s\n",
               cpu_name, poll16 * 5238UL / 100000UL, poll16 * 5238UL / 10000UL % 10UL,
               baud_tab[cpu_top], has_fifo ? "yes" : "no", has_scr ? "yes" : "no",
               !has_fifo ? "n/a" : burst_bi() < 0 ? "cpu" : rate_lbl[burst_bi()],
               wd_lpt ? "wd76c30" : "no");
    for (j = 0; j < 2; j++)
        for (i = 0; i < NCHK && (j ? do_ext : do_int); i++)
            if (chk_st[j][i] != ST_NA)
                log_printf("P1 %s %-24s %s\n", j ? "EXT" : "INT", chk[i].name, st[chk_st[j][i]]);
    log_printf("# P2: bits = bit periods at that rate (8N1 char = 10). burst = idle line, write to DR,\n"
               "# min-max over %d write phases; run = per byte over n back to back (first..last DR),\n# gap = shortest-longest DR to DR inside the run;\n"
               "# lines = worst RTS->CTS/DTR->DSR edge. \n",
               P2_SAMP);
    for (j = 0; j < 2; j++)
        for (i = baud0; i < baud0 + nbaud; i++) {
            q = &p2[j][i];
            if (q->st == ST_PEND)
                continue;
            if (q->n == 0 && !q->cpu) {
                log_printf("P2 %s %6lu TIMEOUT FAIL\n", j ? "EXT" : "INT", baud_tab[i]);
                continue;
            }
            fmt_bits(a[0], q->bmin, i); log_ms(m[0], q->bmin);
            fmt_bits(a[1], q->bmax, i); log_ms(m[1], q->bmax);
            sprintf(a[2], "%lu.%lu", run10(q, i) / 10, run10(q, i) % 10); log_ms(m[2], q->run);
            if (q->cpu) {
                strcpy(a[2], "cpu");    /* RUN skipped: no number for perf_ab to compare */
                strcpy(m[2], "cpu");
            }
            fmt_bits(a[3], q->first, i);
            fmt_bits(a[4], q->gmin, i);
            fmt_bits(a[5], q->gmax, i);
            log_ms(ln, q->line);
            log_printf("P2 %s %6lu bits burst=%s-%s run=%s gap=%s-%s first=%s n=%d | ms burst=%s-%s run=%s lines=%s | %s%s\n",
                       j ? "EXT" : "INT", baud_tab[i], a[0], a[1], a[2], a[4], a[5], a[3], q->n,
                       m[0], m[1], m[2], ln, q->st == ST_PASS ? "ok" : "FAIL", q->slow ? " SLOW" : "");
        }
    log_printf("# P3: ms, %d stop-and-wait rtt per rate, and %d edges per line;\n"
               "# rate rows are the link, bytes 01-FF; jitter = max - min\n"
               "# 0x00 rows: the 0x00 rtts apart, over-link = 0x00 avg - link avg, HOLD when every\n"
               "# 0x00 is over 2 ms (P3_HOLD) past the link max (host held it to check for a break);\n"
               "# all bytes = the row with the 0x00s in\n"
               "# S3: each sample in send order. warm = uncounted rtt \n"
               "# sent-byte:ms, up:/dn:ms per line edge \n", P3_SAMP, P3_SAMP);
    for (j = 0; j < 2; j++) {
        if (!p3_on(j))
            continue;
        for (i = baud0; i < baud0 + nbaud; i++) {
            sprintf(what, "%6lu", baud_tab[i]);
            log_p3(j, what, &p3[j][i]);
            log_s3(j, what, &p3[j][i], 0);
        }
        for (k = 0; k < P3_NLINE; k++) {
            sprintf(what, "LINE %s->%s", p3_lname[j][k][0], p3_lname[j][k][1]);
            log_p3(j, what, &p3l[j][k]);
            log_s3(j, what, &p3l[j][k], 1);
        }
    }
    for (i = 0; i < nnotes; i++)
        log_printf("NOTE %s\n", notes[i]);
    if (notes_dropped)
        log_printf("NOTE ... %d more not kept\n", notes_dropped);
    strcpy(done, "sertest done");
    strupr(done);
    log_printf("%s RESULT=%s %s\n", done, pass ? "PASS" : "FAIL", summary);
}

static char result_line[81];

/* The run's counts on the stage line, the verdict in the footer. */
static void show_result(void)
{
    status(result_line);
    draw_footer();
}

static void show_page(int n)
{
    if (n == 1)
        draw_page1(0);
    else if (n == 2)
        draw_page2(0);
    else if (n <= 4)
        draw_page3(n == 3, 0);
    else
        draw_page5(n == 5);
    show_result();
}

/* Run one page from scratch: 1 function, 2 baud cycle, 4 INT latency,
 * 3 EXT latency. Stops early at an item boundary when a key sets stop_req. */
static void run_pg(int n)
{
    int p = (n == 3);

    cur_run_pg = n;
    notes_drop_pg(n);
    result_word = "RUNNING";
    stage_prev[0] = 0;
    cur_bi = top_bi();
    cur_lcr = 0x03;
    if (n == 1) {
        ext_noplug = 0;
        draw_page1(1);
        if (do_int)
            run_panel(0);
        if (do_ext && !stop_req)
            run_panel(1);
        draw_notes();
    } else if (n == 2) {
        draw_page2(1);
        note_panel = "";
        run_page2();
    } else {
        draw_page3(p, 1);
        if (p3_on(p))
            run_page3(p);
    }
}

/* Counts, SERLOG.TXT and the verdict, from whatever the pages hold now. */
static int finish_run(void)
{
    char sum[96];
    int ip = 0, it = 0, ep = 0, et = 0, rp, rt, rs, lp, lt, ls, nh = 0, pass = 1, p;
    unsigned long hlo, hhi;

    if (do_int) {
        ip = tally(0, &it);
        if (ip != it)
            pass = 0;
    }
    if (do_ext) {
        ep = tally(1, &et);
        if (ep != et || ext_noplug)
            pass = 0;
    }
    rp = rt = rs = 0;
    for (p = 0; p < 2; p++) {
        int a, b, s;
        a = p2_tally(p, &b, &s);
        rp += a;
        rt += b;
        rs += s;
    }
    if (rp != rt)
        pass = 0;
    lp = p3_tally(&lt, &ls);
    for (p = 0; p < 2; p++)
        if (p3_on(p))
            nh += p3_holds(p, &hlo, &hhi);
    if (lp != lt)
        pass = 0;

    sprintf(sum, "P1 INT=%d/%d EXT=%d/%d P2 RATES=%d/%d SLOW=%d P3 LAT=%d/%d WARN=%d HOLD=%d",
            ip, it, ep, et, rp, rt, rs, lp, lt, ls, nh);
    log_open(prog_path);                /* overwrite: this run's log only */
    log_full(pass, sum);
    sprintf(result_line, "P1 INT %d/%d  EXT %d/%d   P2 RATES %d/%d SLOW %d   P3 LAT %d/%d SLOW %d",
            ip, it, ep, et, rp, rt, rs, lp, lt, ls);
    result_word = pass ? "PASS" : "FAIL";
    show_result();
    return pass;
}

/* The page to run for a screen page, 5/6 show 3/4's samples. */
static int run_page_of(int screen)
{
    return screen == 5 ? 3 : screen == 6 ? 4 : screen;
}

/* Run page n; R while it runs starts it over, Esc leaves stop_req set. */
static void run_pg_keys(int n)
{
    do {
        stop_req = STOP_NONE;
        run_pg(n);
    } while (stop_req == STOP_RERUN);
}

/* ---- /STEP: nothing runs until G. Each G runs the next stage and appends
 * a marker line to the log, such as "SERTEST PAGE 2 DONE", which the testlist
 * waits for on the raw floppy image. Markers are upper-cased at run time.
 * The final log rewrite drops them. */
static void step_log(const char *fmt, int n)
{
    char buf[40];
    sprintf(buf, fmt, n);
    strupr(buf);
    log_printf("%s\n", buf);
}

/* Show what G will run on the stage line and wait for it. Esc sets
 * STOP_QUIT. what is the stage name. */
static void step_gate(const char *what)
{
    char line[80];
    int k;

    sprintf(line, "STEP READY - G runs %s", what);
    vfill(STAGE_ROW, 0, 80, ' ', A_NORM);
    vput(STAGE_ROW, 1, A_BRIGHT, line);
    draw_footer();
    for (;;) {
        k = getch();
        if (k == 0 || k == 0xE0)
            (void) getch();
        else if (k == 27) {
            stop_req = STOP_QUIT;
            return;
        } else if (k == 'g' || k == 'G') {
            stop_req = STOP_NONE;
            return;
        }
    }
}

static int run_all(void)
{
    static const int order[4] = { 1, 2, 4, 3 };  /* INT before EXT, ends on page 3 */
    char what[32];
    int i;

    nnotes = notes_dropped = 0;
    ext_noplug = 0;
    result_word = "RUNNING";
    stage_prev[0] = 0;
    log_open(prog_path);                /* overwrite: this run's log only */
    detect_uart();
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(MCR_LOOP);
    detect_typesw();                /* nothing goes out while the poll loop is timed */
    drain_rx();
    cal_poll();

    if (step_mode) {
        draw_page1(1);                  /* the rows page 1 will run, all pending */
        step_log("sertest step ready", 0);
    }
    for (i = 0; i < 4; i++) {
        if (step_mode) {
            sprintf(what, "page %d %s", order[i], page_name[order[i] - 1]);
            step_gate(what);
            if (stop_req == STOP_QUIT)
                return 0;
        }
        run_pg_keys(order[i]);
        if (stop_req == STOP_QUIT)
            return 0;
        if (step_mode)
            step_log("sertest page %d done", order[i]);
        else if (i < 3) {
            t_sleep_keys(2UL * TICKS_1S);   /* each page's verdicts stay up a moment */
            if (stop_req == STOP_QUIT)
                return 0;
            if (stop_req == STOP_RERUN)     /* R during the pause: this page again */
                i--;
        }
    }
    return finish_run();
}

/* ------------------------------------------------------------ dual mode */

/* SERTEST /DUAL : COM1 and COM2 at the same time, written to DUALLOG.TXT.
 *
 * STEP 1 SINGLE: each port alone. UART type, INT data 00-FF and RTS->CTS,
 *   then plug detect, EXT data 00-FF and DTR->DSR. The EXT rows run only if
 *   both ports found a plug, PLUGS=2 in the log. Otherwise they show "--" and
 *   do not fail the run.
 * STEP 2 EXT / STEP 3 INT: DUAL_NCFG rows of lat_secs, both ports at once
 *   from one poll loop. Each port is a lane that never waits for the other.
 *     data   stop-and-wait round trips on the lane's own stream. COM1 PRBS-9,
 *            COM2 PRBS-11, 512 bytes each, masked to the row's word length.
 *            The rows give the ports different rates and formats.
 *     lines  every few bytes the lane flips RTS or DTR, COM1 every 5 starting
 *            with RTS and COM2 every 7 starting with DTR, and times CTS/DSR.
 *     break  every 48 bytes, COM2 offset by 24, the lane holds a break for
 *            DUAL_BRK_CH character times and wants one 0x00 with BI back.
 *            Not with /NOBRK on EXT.
 *   Cross-talk is the point. A delta bit the lane did not cause, CTS/DSR
 *   changing with no toggle pending, BI/FE/PE/OE outside its own break, or a
 *   byte out of stream fails the row. RTT is in bit periods of the lane's
 *   rate. SLOW is RTT over a frame + 2 bits or a line over 2 ms, reported
 *   not failed.
 *
 * The last DUALLOG line is "SERTEST DUAL DONE RESULT=...", upper-cased at run
 * time. /STEP gates the three steps on G with "SERTEST DUAL STEP n DONE"
 * markers. */

#define DPAT_N       512
#define DUAL_NCFG    6
#define DUAL_BRK_CH  20         /* break hold, character times */
#define DUAL_BRK_EV  48         /* bytes between breaks, per lane */
#define DUAL_GRACE   (20UL * TICKS_1MS)     /* own deltas allowed after a follow */
#define DUAL_TOP     4          /* first table row on a dual page */
#define DUAL_NOTE    17         /* failure-notes box top */

struct dual_cfg {
    unsigned char bi[2];        /* baud_tab index per lane */
    unsigned char lcr[2];
};

/* 7E1 = 1Ah, 8O2 = 0Fh, 5N1.5 = 04h */
static const struct dual_cfg dual_cfg[DUAL_NCFG] = {
    { { 11, 11 }, { 0x03, 0x03 } },     /* 115200 8N1 | 115200 8N1 */
    { {  7,  7 }, { 0x03, 0x03 } },     /*   9600 8N1 |   9600 8N1 */
    { {  4,  4 }, { 0x03, 0x03 } },     /*   1200 8N1 |   1200 8N1 */
    { { 11,  7 }, { 0x03, 0x1A } },     /* 115200 8N1 |   9600 7E1 */
    { {  7, 11 }, { 0x1A, 0x03 } },     /*   9600 7E1 | 115200 8N1 */
    { { 10,  9 }, { 0x0F, 0x04 } }      /*  57600 8O2 |  38400 5N1.5 */
};

#define LN_SEND     0
#define LN_WAIT     1
#define LN_BRK      2
#define LN_BRKWAIT  3
#define LN_BRKQUIET 4

struct lane {
    /* set per run */
    int           k;            /* 0 COM1, 1 COM2 */
    unsigned int  base;
    /* set per row */
    unsigned char bi, lcr, mask, mcr, ext, brk_on;
    int           st, ending;
    unsigned int  pos, nbytes;
    unsigned char sent;
    unsigned long t0, frame, bound;
    unsigned char lpend, lwant, dexp, last_msr;
    unsigned long lt0, dexp_t;
    unsigned int  nexttog;
    int           togdtr;       /* next toggle: 1 DTR, 0 RTS */
    /* results */
    unsigned long n, sum, min, max, nl, lmax;
    unsigned int  bad, lsrerr, tmo, linefail, spur, brk_ok, brk_n;
    unsigned char slow;
};

struct dual_res {
    int           run;          /* 0 not run, 1 run, 2 skipped, no plug */
    struct lane   l[2];
};

static unsigned char dpat[2][DPAT_N];
static struct dual_res dres[2][DUAL_NCFG];     /* [0 INT, 1 EXT] */
static unsigned int  dbase[2];
static const char   *duart[2] = { "?", "?" };
static int           dfifo[2];
static int           dsingle[2][3];   /* [port][INT ok, plug, EXT ok]: -1 not run */
static int           dplugs;
static int           dual_page = 1;  /* 1 EXT, 2 INT */

/* Both lane streams, made once. PRBS-9 for COM1 and PRBS-11 for COM2,
 * 8 bits a byte, MSB first. */
static void dual_gen(void)
{
    unsigned int s, fb, m;
    unsigned char v;
    int k, i, b;

    for (k = 0; k < 2; k++) {
        m = k ? 0x7FF : 0x1FF;
        s = m;
        for (i = 0; i < DPAT_N; i++) {
            v = 0;
            for (b = 0; b < 8; b++) {
                fb = k ? ((s >> 10) ^ (s >> 8)) & 1 : ((s >> 8) ^ (s >> 4)) & 1;
                s = ((s << 1) | fb) & m;
                v = (unsigned char) ((v << 1) | fb);
            }
            dpat[k][i] = v;
        }
    }
}

static void dual_rate(char *buf, const struct lane *l)
{
    char f[8];
    fmt_name(f, l->lcr);
    sprintf(buf, "%-5s %-5s", rate_lbl[l->bi], f);
}

static int lane_errs(const struct lane *l)
{
    return l->bad + l->lsrerr + l->tmo + l->linefail + l->spur + (l->brk_n - l->brk_ok);
}

static int lane_ok(const struct lane *l)
{
    return l->n > 0 && lane_errs(l) == 0;
}

static void dual_header(void)
{
    char buf[81];
    cls();
    sprintf(buf, " XP5 SERTEST  DUAL PAGE %d/2 %s  COM1 %03Xh %-6s  COM2 %03Xh %-6s  CPU %s",
            dual_page, dual_page == 1 ? "EXT" : "INT", dbase[0], duart[0], dbase[1], duart[1], cpu_name);
    vput(0, 0, A_NORM, buf);
    draw_footer();
}

static void dual_single_line(void)
{
    static const char *w[] = { "--", "FAIL", "ok" };
    char buf[80];
    int k, c = 2;

    vput(2, c, A_NORM, "SINGLE");
    for (k = 0; k < 2; k++) {
        sprintf(buf, "COM%d: INT %-4s plug %-3s EXT %-4s", k + 1,
                w[dsingle[k][0] + 1], dsingle[k][1] < 0 ? "--" : dsingle[k][1] ? "yes" : "no",
                w[dsingle[k][2] + 1]);
        vput(2, c + 8 + k * 35, A_NORM, buf);
    }
}

/* One table row: the RATE FMT column is dual_rate's 11 characters. */
#define DUAL_FMT "%d COM%d %s %6lu %5s %5s %5s %4lu %-7s %-5s %3d %s"

static void dual_row(int p, int i)
{
    const struct dual_res *q = &dres[p][i];
    const struct lane *l;
    char buf[81], rt[16], mn[8], av[8], mx[8], lm[12], br[8];
    int k, r;

    for (k = 0; k < 2; k++) {
        r = DUAL_TOP + i * 2 + k;
        vfill(r, 1, 78, ' ', A_NORM);
        l = &q->l[k];
        if (q->run == 0 || q->run == 2) {
            sprintf(buf, "%d COM%d %s", i + 1, k + 1, q->run == 2 ? "-- no plug on both ports" : "");
            vput(r, 2, A_NORM, buf);
            continue;
        }
        dual_rate(rt, l);
        if (l->n) {
            fmt_bits(mn, l->min, l->bi);
            fmt_bits(av, l->sum / l->n, l->bi);
            fmt_bits(mx, l->max, l->bi);
        } else
            strcpy(mn, "--"), strcpy(av, "--"), strcpy(mx, "--");
        if (l->nl)
            fmt_ms(lm, l->lmax, 0);
        else
            strcpy(lm, "--");
        if (!l->brk_on)
            strcpy(br, l->ext && no_brk ? "n/c" : "--");
        else
            sprintf(br, "%u/%u", l->brk_ok, l->brk_n);
        sprintf(buf, DUAL_FMT "%s", i + 1, k + 1, rt,
                l->n, mn, av, mx, l->nl, lm, br, lane_errs(l),
                q->run == 1 && l->st < 0 ? "RUN" : lane_ok(l) ? "PASS" : "FAIL", l->slow ? " *" : "");
        vput(r, 2, (lane_ok(l) || l->st < 0) ? A_NORM : A_BRIGHT, buf);
    }
}

static void draw_dual(void)
{
    int i, p = dual_page == 1;
    char buf[80];

    cur_page = dual_page;
    dual_header();
    sprintf(buf, "DUAL %s \xC4\xC4 COM1 + COM2 at once, %d s per row", p ? "EXT (plugs)" : "INT (MCR loop)", lat_secs);
    box(1, DUAL_NOTE + 4, buf);
    dual_single_line();
    sprintf(buf, "# PORT %-11s %6s %5s %5s %5s %4s %-7s %-5s %3s %s",
            "RATE  FMT", "BYTES", "MIN", "AVG", "MAX", "EDGE", "LINE", "BRK", "ERR", "RESULT");
    vput(3, 2, A_NORM, buf);
    box_rule(DUAL_NOTE, "FAILURE NOTES");
    for (i = 0; i < DUAL_NCFG; i++)
        dual_row(p, i);
    vput(LEG_ROW, 1, A_NORM, "MIN AVG MAX: round trip, bit periods at that port's rate. LINE: worst ms. *=slow");
    draw_notes_at(DUAL_NOTE, 3);
}

/* Note text: which port, rate and format, then the detail. */
static void lane_note(const struct lane *l, const char *what, unsigned int a, unsigned int b)
{
    char rt[16];
    dual_rate(rt, l);
    note("COM%d %s #%u %s %02X %02X", l->k + 1, rt, l->nbytes, what, a, b);
}

/* One pass for one lane: MSR, then one step of its data state machine.
 * Never blocks. */
static void lane_step(struct lane *l, unsigned long now)
{
    unsigned char msr, lsr, d, dl, chg, want;
    unsigned long t;

    msr = (unsigned char) inp(l->base + REG_MSR);
    dl = (unsigned char) (msr & 0x0F);
    if (l->dexp_t && now >= l->dexp_t) {
        l->dexp = 0;
        l->dexp_t = 0;
    }
    if (dl & ~l->dexp) {
        if (l->spur++ < 2)
            lane_note(l, "stray MSR delta (MSR,ok)", msr, l->dexp);
    }
    if (l->lpend) {
        if ((msr & l->lpend) == l->lwant) {
            t = now - l->lt0;
            l->nl++;
            if (t > l->lmax)
                l->lmax = t;
            l->lpend = 0;
            l->dexp_t = now + DUAL_GRACE;
        } else if (now - l->lt0 >= TMO_TICKS) {
            if (l->linefail++ < 2)
                lane_note(l, "line no follow (MSR,want)", msr, l->lwant);
            l->lpend = 0;
            l->dexp_t = now + DUAL_GRACE;
        }
    }
    /* CTS/DSR must match what the lane drives, except the one it waits on.
     * Counted once per change. */
    want = (unsigned char) (((l->mcr & MCR_RTS) ? MSR_CTS : 0) | ((l->mcr & MCR_DTR) ? MSR_DSR : 0));
    chg = (unsigned char) ((msr ^ l->last_msr) & (MSR_CTS | MSR_DSR) & ~l->lpend);
    if (chg && ((msr ^ want) & chg)) {
        if (l->spur++ < 2)
            lane_note(l, "CTS/DSR moved (MSR,MCR)", msr, l->mcr);
    }
    l->last_msr = msr;

    switch (l->st) {
    case LN_SEND:
        if (l->ending)
            break;
        lsr = (unsigned char) inp(l->base + REG_LSR);
        if (!(lsr & LSR_THRE))
            break;
        if (lsr & LSR_DR) {
            d = (unsigned char) inp(l->base + REG_DATA);
            if (l->bad++ < 2)
                lane_note(l, "stray byte (got,LSR)", d, lsr);
            break;
        }
        l->sent = (unsigned char) (dpat[l->k][l->pos] & l->mask);
        l->t0 = t_now();                /* the clock right at the write */
        outp(l->base + REG_DATA, l->sent);
        l->st = LN_WAIT;
        break;

    case LN_WAIT:
        lsr = (unsigned char) inp(l->base + REG_LSR);
        if (lsr & LSR_DR) {
            t = t_now() - l->t0;        /* right at DR */
            d = (unsigned char) inp(l->base + REG_DATA);
            if (lsr & (LSR_OE | 0x04 | 0x08 | LSR_BI)) {
                if (l->lsrerr++ < 2)
                    lane_note(l, "LSR error (LSR,got)", lsr, d);
            } else if ((unsigned char) (d & l->mask) != l->sent) {
                if (l->bad++ < 2)
                    lane_note(l, "wrong byte (sent,got)", l->sent, d);
            } else {
                l->n++;
                l->sum += t;
                if (t < l->min)
                    l->min = t;
                if (t > l->max)
                    l->max = t;
            }
            l->pos = (l->pos + 1) % DPAT_N;
            l->nbytes++;
            l->st = LN_SEND;
            if (!l->lpend && !l->ending && l->nbytes >= l->nexttog) {
                unsigned char bit = (unsigned char) (l->togdtr ? MCR_DTR : MCR_RTS);
                l->mcr ^= bit;
                l->lpend = (unsigned char) (l->togdtr ? MSR_DSR : MSR_CTS);
                l->lwant = (unsigned char) ((l->mcr & bit) ? l->lpend : 0);
                l->dexp |= (unsigned char) (l->togdtr ? (MSR_DDSR | MSR_DDCD | MSR_TERI) : MSR_DCTS);
                l->dexp_t = 0;
                l->lt0 = now;
                outp(l->base + REG_MCR, l->mcr);
                l->nexttog = l->nbytes + (l->k ? 7 : 5);
                l->togdtr = !l->togdtr;
            }
            if (l->brk_on && !l->ending && l->nbytes % DUAL_BRK_EV == (l->k ? DUAL_BRK_EV / 2 : 0)) {
                l->brk_n++;
                l->t0 = now;
                outp(l->base + REG_LCR, l->lcr | 0x40);
                l->st = LN_BRK;
            }
        } else if (now - l->t0 >= TMO_TICKS) {
            if (l->tmo++ < 2)
                lane_note(l, "timeout (sent,LSR)", l->sent, lsr);
            l->pos = (l->pos + 1) % DPAT_N;
            l->nbytes++;
            l->st = LN_SEND;
        }
        break;

    case LN_BRK:
        if (now - l->t0 >= DUAL_BRK_CH * l->frame) {
            outp(l->base + REG_LCR, l->lcr);
            l->t0 = now;
            l->st = LN_BRKWAIT;
        }
        break;

    case LN_BRKWAIT:
        lsr = (unsigned char) inp(l->base + REG_LSR);
        if (lsr & LSR_DR) {
            d = (unsigned char) inp(l->base + REG_DATA);
            if ((lsr & LSR_BI) && d == 0)
                l->brk_ok++;
            else
                lane_note(l, "bad break (got,LSR)", d, lsr);
            l->t0 = now;
            l->st = LN_BRKQUIET;
        } else if (now - l->t0 >= TICKS_100MS + 4 * l->frame) {
            lane_note(l, "break lost (LSR,LCR)", lsr, l->lcr);
            l->st = LN_SEND;
        }
        break;

    case LN_BRKQUIET:              /* a real UART gives ONE 0x00 for a break */
        lsr = (unsigned char) inp(l->base + REG_LSR);
        if (lsr & LSR_DR) {
            d = (unsigned char) inp(l->base + REG_DATA);
            if (l->brk_ok)
                l->brk_ok--;
            lane_note(l, "2nd char on break (got,LSR)", d, lsr);
        }
        if (now - l->t0 >= 4 * l->frame + TICKS_1MS)
            l->st = LN_SEND;
        break;
    }
}

/* Set one lane up for row i: rate, format, FIFO off, MCR quiet, loopback
 * on INT, then wait for CTS/DSR low. */
static void lane_init(struct lane *l, int p, int i)
{
    unsigned int div;
    int k = l->k;
    unsigned int base = l->base;

    memset(l, 0, sizeof(*l));
    l->k = k;
    l->base = base;
    l->bi = dual_cfg[i].bi[k];
    l->lcr = dual_cfg[i].lcr[k];
    l->mask = (unsigned char) ((1 << (5 + (l->lcr & 3))) - 1);
    l->ext = (unsigned char) p;
    l->brk_on = (unsigned char) !(p && no_brk);
    l->mcr = (unsigned char) (p ? 0 : MCR_LOOP);
    l->frame = frame_ticks(l->bi, l->lcr);
    l->bound = l->frame + 2UL * bit64(l->bi) / 64 + P2_SLACK + 2UL * poll16 / 16;
    l->min = 0xFFFFFFFFUL;
    l->pos = (unsigned int) (k * 101);
    l->nexttog = k ? 7 : 5;
    l->togdtr = k;
    l->st = -1;                 /* not started: shows RUN until the row ends */
    div = div_tab[l->bi];
    outp(base + REG_IER, 0x00);
    outp(base + REG_LCR, 0x80);
    outp(base + REG_DLL, div & 0xFF);
    outp(base + REG_DLH, div >> 8);
    outp(base + REG_LCR, l->lcr);
    outp(base + REG_FCR, 0x00);
    outp(base + REG_MCR, l->mcr);
}

static void lane_settle(struct lane *l)
{
    int n = 0;
    t_start();
    while ((inp(l->base + REG_MSR) & (MSR_CTS | MSR_DSR)) && t_now() < TICKS_100MS)
        ;
    while ((inp(l->base + REG_LSR) & LSR_DR) && n++ < 64)
        (void) inp(l->base + REG_DATA);
    (void) inp(l->base + REG_LSR);
    l->last_msr = (unsigned char) inp(l->base + REG_MSR);    /* clears the deltas */
}

static void dual_one(int p, int i)
{
    struct dual_res *q = &dres[p][i];
    struct lane *a = &q->l[0], *b = &q->l[1];
    unsigned long now, len = (unsigned long) lat_secs * TICKS_1S;
    unsigned int pass = 0;
    char name[32];
    int k;

    a->k = 0; a->base = dbase[0];
    b->k = 1; b->base = dbase[1];
    lane_init(a, p, i);
    lane_init(b, p, i);
    q->run = 1;
    sprintf(name, "%s ROW %d", p ? "EXT" : "INT", i + 1);
    stage(name, "TESTING");
    if (cur_page == (p ? 1 : 2))
        dual_row(p, i);
    t_sleep(RATE_SETTLE);
    lane_settle(a);
    lane_settle(b);
    a->st = b->st = LN_SEND;

    t_start();
    for (;;) {
        now = t_now();
        if (now >= len)
            a->ending = b->ending = 1;
        lane_step(a, now);
        lane_step(b, now);
        if (a->ending && a->st == LN_SEND && !a->lpend && b->st == LN_SEND && !b->lpend)
            break;
        if (now >= len + 2UL * TICKS_1S)
            break;                      /* an outstanding byte or edge that never came */
        if ((++pass & 0x3FF) == 0) {
            key_poll();
            if (stop_req == STOP_QUIT)
                break;
            if (cur_page == (p ? 1 : 2) && (pass & 0x3FFF) == 0)
                progress((int) (now / TICKS_100MS), (int) (len / TICKS_100MS));
        }
    }
    for (k = 0; k < 2; k++) {
        struct lane *l = &q->l[k];
        if (l->lpend && l->linefail++ < 2)
            lane_note(l, "line pending at end", l->lpend, l->lwant);
        if (l->n == 0)
            lane_note(l, "no round trip", 0, 0);
        l->slow = (unsigned char) (l->n && (l->sum / l->n > l->bound || l->lmax > 2UL * TICKS_1MS));
        outp(l->base + REG_LCR, l->lcr);
        outp(l->base + REG_MCR, (unsigned char) (p ? 0 : MCR_LOOP));
    }
    stage(name, lane_ok(a) && lane_ok(b) ? "PASS" : "FAIL");
    if (cur_page == (p ? 1 : 2)) {
        dual_row(p, i);
        draw_notes_at(DUAL_NOTE, 3);
    }
}

static void dual_panel(int p)
{
    int i;
    note_panel = p ? "EXT" : "INT";
    dual_page = p ? 1 : 2;
    draw_dual();
    for (i = 0; i < DUAL_NCFG && stop_req != STOP_QUIT; i++) {
        if (p && dplugs < 2)
            dres[p][i].run = 2;
        else
            dual_one(p, i);
        dual_row(p, i);
    }
}

/* Step 1: each port alone with the full test's single-port checks. */
static void dual_single(void)
{
    int k;
    char name[24];

    dual_page = 1;
    draw_dual();
    dplugs = 0;
    for (k = 0; k < 2 && stop_req != STOP_QUIT; k++) {
        port = dbase[k];
        com_n = k + 1;
        irq = k ? 3 : 4;
        note_panel = "INT";
        sprintf(name, "COM%d SINGLE", k + 1);
        stage(name, "TESTING");
        cur_bi = top_bi();
        cur_lcr = 0x03;
        uart_init(MCR_LOOP);
        dsingle[k][0] = data_test(MCR_LOOP) && line_test(MCR_LOOP, MCR_RTS, MSR_CTS, 4, NULL);
        if (!dsingle[k][0])
            note("COM%d data 00-FF or RTS->CTS failed", k + 1);
        note_panel = "EXT";
        uart_init(0x00);
        t_sleep(TICKS_100MS);
        dsingle[k][1] = line_test(0x00, MCR_RTS, MSR_CTS, 2, NULL);
        if (dsingle[k][1]) {
            dplugs++;
            dsingle[k][2] = data_test(0x00) && line_test(0x00, MCR_DTR, MSR_DSR, 4, NULL);
            if (!dsingle[k][2])
                note("COM%d data 00-FF or DTR->DSR failed", k + 1);
        } else
            note("COM%d no plug: RTS never came back on CTS", k + 1);
        uart_init(0x00);
        stage(name, dsingle[k][0] && (!dsingle[k][1] || dsingle[k][2]) ? "PASS" : "FAIL");
        dual_single_line();
        draw_notes_at(DUAL_NOTE, 3);
    }
    port = dbase[0];
    com_n = 1;
    irq = 4;
}

static int dual_tally(int p, int *total)
{
    int i, k, ok = 0;
    *total = 0;
    for (i = 0; i < DUAL_NCFG; i++)
        if (dres[p][i].run == 1)
            for (k = 0; k < 2; k++) {
                (*total)++;
                ok += lane_ok(&dres[p][i].l[k]);
            }
    return ok;
}

static int dual_slow(void)
{
    int p, i, k, n = 0;
    for (p = 0; p < 2; p++)
        for (i = 0; i < DUAL_NCFG; i++)
            if (dres[p][i].run == 1)
                for (k = 0; k < 2; k++)
                    n += dres[p][i].l[k].slow;
    return n;
}

static void dual_log(int pass, const char *sum)
{
    static const char *w[] = { "n/a", "FAIL", "ok" };
    char rt[16], mn[8], av[8], mx[8], lm[12], done[24];
    const struct lane *l;
    int p, i, k;

    log_printf("XP5 SERTEST 0.1 dual test\n");
    log_printf("HW CPU %s poll=%lu.%luus COM1 %03Xh %s fifo=%s COM2 %03Xh %s fifo=%s\n", cpu_name,
               poll16 * 5238UL / 100000UL, poll16 * 5238UL / 10000UL % 10UL,
               dbase[0], duart[0], dfifo[0] ? "yes" : "no", dbase[1], duart[1], dfifo[1] ? "yes" : "no");
    for (k = 0; k < 2; k++)
        log_printf("SINGLE COM%d INT=%s PLUG=%s EXT=%s\n", k + 1, w[dsingle[k][0] + 1],
                   dsingle[k][1] < 0 ? "n/a" : dsingle[k][1] ? "yes" : "no", w[dsingle[k][2] + 1]);
    log_printf("# D rows: %d s of both ports at once; rtt = stop-and-wait THR write to DR in bit periods\n"
               "# of that port's rate, min/avg/max; edges = RTS/DTR toggles followed, line = worst follow ms;\n"
               "# brk = breaks back as one 0x00+BI / sent; err = wrong/stray bytes + LSR errors + timeouts\n"
               "# + lines not followed + spurious MSR (cross-talk) + bad breaks\n", lat_secs);
    for (p = 1; p >= 0; p--)
        for (i = 0; i < DUAL_NCFG; i++) {
            if (dres[p][i].run != 1) {
                if (dres[p][i].run == 2)
                    log_printf("D %s %d SKIP no plug on both ports\n", p ? "EXT" : "INT", i + 1);
                continue;
            }
            for (k = 0; k < 2; k++) {
                l = &dres[p][i].l[k];
                dual_rate(rt, l);
                if (l->n) {
                    fmt_bits(mn, l->min, l->bi);
                    fmt_bits(av, l->sum / l->n, l->bi);
                    fmt_bits(mx, l->max, l->bi);
                } else
                    strcpy(mn, "-"), strcpy(av, "-"), strcpy(mx, "-");
                log_ms(lm, l->lmax);
                log_printf("D %s %d COM%d %s n=%lu rtt=%s/%s/%s edges=%lu line=%s brk=%u/%u bad=%u lsr=%u tmo=%u linefail=%u spur=%u | %s%s\n",
                           p ? "EXT" : "INT", i + 1, k + 1, rt, l->n, mn, av, mx, l->nl, lm,
                           l->brk_ok, l->brk_n, l->bad, l->lsrerr, l->tmo, l->linefail, l->spur,
                           lane_ok(l) ? "ok" : "FAIL", l->slow ? " SLOW" : "");
            }
        }
    for (i = 0; i < nnotes; i++)
        log_printf("NOTE %s\n", notes[i]);
    if (notes_dropped)
        log_printf("NOTE ... %d more not kept\n", notes_dropped);
    strcpy(done, "sertest dual done");
    strupr(done);
    log_printf("%s RESULT=%s %s\n", done, pass ? "PASS" : "FAIL", sum);
}

static int dual_finish(void)
{
    char sum[96];
    int ip, it, ep, et, pass, k;

    ip = dual_tally(0, &it);
    ep = dual_tally(1, &et);
    pass = ip == it && ep == et && it > 0;
    for (k = 0; k < 2; k++)
        if (dsingle[k][0] != 1 || (dsingle[k][1] == 1 && dsingle[k][2] != 1))
            pass = 0;
    sprintf(sum, "PLUGS=%d INT=%d/%d EXT=%d/%d SLOW=%d", dplugs, ip, it, ep, et, dual_slow());
    log_open(prog_path);                /* overwrite: this run's log only */
    dual_log(pass, sum);
    sprintf(result_line, "DUAL  PLUGS %d   INT %d/%d   EXT %d/%d%s   SLOW %d",
            dplugs, ip, it, ep, et, dplugs < 2 ? " (skipped)" : "", dual_slow());
    result_word = pass ? "PASS" : "FAIL";
    show_result();
    return pass;
}

static int run_dual(void)
{
    static const char *what[3] = { "step 1 SINGLE (each port alone)", "step 2 EXT (both at once)",
                                   "step 3 INT (both at once)" };
    static const unsigned int defb[2] = { 0x3F8, 0x2F8 };
    unsigned int far *bda = (unsigned int far *) MK_FP(0x0040, 0x0000);
    int k, s;

    log_name = "DUALLOG.TXT";
    log_open(prog_path);
    nnotes = notes_dropped = 0;
    result_word = "RUNNING";
    for (k = 0; k < 2; k++) {
        dbase[k] = bda[k] ? bda[k] : defb[k];
        port = dbase[k];
        detect_uart();
        duart[k] = uart_name;
        dfifo[k] = has_fifo;
        dsingle[k][0] = dsingle[k][1] = dsingle[k][2] = -1;
    }
    port = dbase[0];
    uart_name = duart[0];
    has_fifo = dfifo[0];
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(MCR_LOOP);
    drain_rx();
    cal_poll();
    dual_gen();
    memset(dres, 0, sizeof(dres));
    dual_page = 1;
    draw_dual();
    if (step_mode)
        step_log("sertest dual ready", 0);

    for (s = 0; s < 3; s++) {
        if (step_mode) {
            step_gate(what[s]);
            if (stop_req == STOP_QUIT)
                return 0;
        }
        stop_req = STOP_NONE;
        if (s == 0)
            dual_single();
        else
            dual_panel(s == 1);
        if (stop_req == STOP_QUIT)
            return 0;
        if (step_mode)
            step_log("sertest dual step %d done", s + 1);
    }
    /* The INT page stays up for the last step's screenshot. */
    return dual_finish();
}

static int parse_args(int argc, char **argv)
{
    unsigned long b;
    int i, j;
    char *a;
    for (i = 1; i < argc; i++) {
        a = strupr(argv[i]);
        if (strncmp(a, "/B:", 3) == 0) {
            b = strtoul(a + 3, NULL, 10);
            for (j = 0; j < NBAUD && baud_tab[j] != b; j++)
                ;
            if (j == NBAUD)
                return 0;
            baud0 = j;
            nbaud = 1;
        } else if (strncmp(a, "/COM:", 5) == 0) {
            com_n = atoi(a + 5);
            if (com_n < 1 || com_n > 4)
                return 0;
        } else if (strcmp(a, "/INT") == 0) {
            do_ext = 0;
        } else if (strcmp(a, "/EXT") == 0) {
            do_int = 0;
        } else if (strcmp(a, "/NORI") == 0) {
            no_ri = 1;
        } else if (strcmp(a, "/NOBRK") == 0) {
            no_brk = 1;
        } else if (strcmp(a, "/STEP") == 0) {
            step_mode = 1;
        } else if (strcmp(a, "/DUAL") == 0) {
            dual_mode = 1;
        } else if (strcmp(a, "/LAT") == 0) {
            lat_mode = 1;
        } else if (strncmp(a, "/LAT:", 5) == 0) {
            if (strlen(a + 5) == 0 || strlen(a + 5) >= sizeof(lat_arg))
                return 0;
            lat_mode = 1;
            strcpy(lat_arg, a + 5);
        } else if (strncmp(a, "/T:", 3) == 0) {
            lat_secs = atoi(a + 3);
            if (lat_secs < 1 || lat_secs > 60)
                return 0;
        } else
            return 0;
    }
    if (dual_mode)
        return !lat_mode && com_n == 1 && nbaud == NBAUD;   /* COM1 + COM2 */
    if (lat_mode) {
        if (!do_int && !do_ext)
            return 0;
        if (do_int && do_ext)
            do_int = 0;             /* latency mode tests one side: EXT unless /INT */
        return lat_select();
    }
    return do_int || do_ext;
}

int main(int argc, char **argv)
{
    static const unsigned int defbase[4] = { 0x3F8, 0x2F8, 0x3E8, 0x2E8 };
    unsigned int far *bda = (unsigned int far *) MK_FP(0x0040, 0x0000);
    int pass, k;

    if (!parse_args(argc, argv)) {
        printf("usage: SERTEST [/B:baud] [/COM:1-4] [/INT] [/EXT] [/NORI] [/NOBRK] [/STEP]\n"
               "       SERTEST /LAT[:lines] [/B:baud] [/INT|/EXT] [/T:secs] [/COM:1-4] [/NORI]\n"
               "       SERTEST /DUAL [/T:secs] [/NOBRK] [/STEP]\n"
               "  baud:  110 150 300 600 1200 2400 4800 9600 19200 38400 57600 115200\n"
               "  lines: ALL or a list , /LAT:RTS or /LAT:RTS,DTR\n"
               "         EXT: DATA RTS DTR DCD RI    INT: DATA RTS DTR OUT1 OUT2\n"
               "  /LAT without /B starts at 110 \n");
        return 2;
    }
    prog_path = argv[0];
    port = bda[com_n - 1] ? bda[com_n - 1] : defbase[com_n - 1];
    irq = (com_n & 1) ? 4 : 3;

    vram = (*(unsigned char far *) MK_FP(0x0040, 0x0049) == 7)
           ? (unsigned char far *) MK_FP(0xB000, 0)
           : (unsigned char far *) MK_FP(0xB800, 0);
    cursor(0);
    pit_mode(0x34);
    clk_init();
    detect_cpu();

    if (lat_mode) {
        detect_uart();
        log_open(prog_path);
        run_latency();
        pass = 1;
    } else if (dual_mode) {
        pass = run_dual();
        k = stop_req == STOP_QUIT ? 27 : 0;
        while (k != 27 && kbhit())
            (void) getch();
        while (k != 27) {
            k = getch();
            if (k == '1' || k == '2' || k == 13) {
                dual_page = k == 13 ? 3 - dual_page : k - '0';
                draw_dual();
                show_result();
            }
        }
        if (result_word[0] == 'R')
            pass = 0;                   /* stopped before the verdict */
        port = dbase[1];                /* COM2 back to a plain port; COM1 below */
        cur_bi = top_bi();
        cur_lcr = 0x03;
        uart_init(MCR_DTR | MCR_RTS);
        port = dbase[0];
    } else {
        pass = run_all();
        if (stop_req == STOP_QUIT)
            k = 27;
        else {
            while (kbhit())
                (void) getch();
            k = 0;
        }
        while (k != 27) {
            k = getch();
            if (k == 27)
                break;
            if (k == 'r' || k == 'R') {
                /* rerun the page on screen, then recount everything */
                run_pg_keys(run_page_of(cur_page));
                if (stop_req == STOP_QUIT)
                    break;
                pass = finish_run();
                show_page(cur_page);
            } else if (k >= '1' && k < '1' + NPAGE && k - '0' != cur_page)
                show_page(k - '0');
            else if (k == 13)
                show_page(cur_page % NPAGE + 1);
        }
        if (stop_req == STOP_QUIT && result_word[0] == 'R')
            pass = 0;                   /* stopped before the verdict */
    }

    pit_mode(0x36);
    cur_bi = top_bi();
    cur_lcr = 0x03;
    uart_init(MCR_DTR | MCR_RTS);
    cls();
    cursor(1);
    return pass ? 0 : 1;
}
