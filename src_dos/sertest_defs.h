/*
 * sertest_defs.h -- constants, tables, shared state and timing helpers of the
 * SERTEST UART checks. Portable C89: no screen, keyboard or OS calls; the machine
 * comes in through sertest_hw.h. Like sertest_core.c it is included into the
 * program's one translation unit (it defines statics, and HW_IMPL below pulls the
 * hardware layer's function bodies in at this spot).
 *
 * SERTEST_FREESTANDING: defined by a port with no C library; it supplies NULL, size_t, va_list,
 * memset, memcpy, strcpy, strncpy, strlen, strcmp, sprintf and vsprintf itself.
 * HW_SCREEN_VAR(v): the DOS build keeps its text-mode video pointer here (as it
 * always did); targets without a screen define it as nothing.
 */
#ifndef SERTEST_DEFS_H
#define SERTEST_DEFS_H

#ifndef SERTEST_FREESTANDING
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#endif
#include "sertest_hw.h"

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

#define PIT_HZ    HW_TICK_HZ

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
HW_SCREEN_VAR(vram)

static volatile unsigned int  isr_hits;
static volatile int           isr_data;
static volatile unsigned char isr_iir;
static volatile unsigned char isr_lsr;
static HW_ISR_PTR(old_isr);

/* ------------------------------------------------------------------ PIT */

static unsigned int  t_last;
static unsigned long t_acc;

#define HW_IMPL
#include "sertest_hw.h"

/* Every PIT read also advances the latency pass clock p_acc. */
static unsigned long p_acc;

static unsigned int pit_step(void)
{
    unsigned int c = hw_timer_read(), d = (unsigned int) ((t_last - c) & 0xFFFFU);   /* 16-bit wrap on any int width */
    t_last = c;
    p_acc += d;
    return d;
}

static void clk_init(void)
{
    t_last = hw_timer_read();
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
    hw_lptr t = (hw_lptr) hw_bda(0x006C);
    unsigned long v;
    hw_irq_off();
    v = *t;
    hw_irq_on();
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


#endif
