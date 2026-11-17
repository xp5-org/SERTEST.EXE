/*
 * sertest_core_p2.c -- page 2's measurement model: the per-rate result record,
 * bit-period arithmetic (bit64, bits10, run10) and wait_idle. Portable C89; included
 * after sertest_core.c, which uses bit64 and wait_idle.
 */
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
    while ((hw_in(port + REG_LSR) & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT))
        if (t_now() >= TMO_TICKS)
            return 0;
    drain_rx();
    return 1;
}
