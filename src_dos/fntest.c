/*
 * fntest.c : COM1 regression tool for 86Box char_fujinet.c, run against
 * fujinet_module/fujinet_loopback.py, a host TCP listener that echoes
 * every byte back.
 *
 * BoIP is a byte pipe only. Modem control is a no-op and status reports
 * CTS, DSR and DCD while the TCP link is up. A loopback plug diagnostic
 * cannot pass, so this tool checks what the path does carry.
 *
 * Modes, argv[1]:
 *   status  : CTS/DSR/DCD set and RI clear when connected, unchanged for
 *             every DTR/RTS combination.
 *   binary  : all 256 byte values echoed at 1200..115200 baud, 8N1.
 *   stream  : 1024 bytes pipelined, FIFO on at 115200 and off at 9600.
 *   wordlen : 7E1 and 5N1 word-length masking.
 *   all     : status, binary, stream, wordlen in order.
 *   drop    : prints DROP READY, host kills the listener, prints DROP SEEN,
 *             host restarts it, then echoes 64 bytes and prints DROP OK.
 *
 * Every mode ends on one "<MODE> OK" or "<MODE> FAIL ..." line for OCR.
 * Exit code is 0 on success and 1 on failure.
 *
 * Build: wcl -0 -ms fntest.c
 * Strict C89: declarations lead their block, block comments only.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <conio.h>
#include <dos.h>

#define REG_DATA  0
#define REG_IER   1
#define REG_DLL   0
#define REG_DLH   1
#define REG_FCR   2
#define REG_LCR   3
#define REG_MCR   4
#define REG_LSR   5
#define REG_MSR   6

#define LSR_DR    0x01
#define LSR_OE    0x02
#define LSR_PE    0x04
#define LSR_FE    0x08
#define LSR_BI    0x10
#define LSR_THRE  0x20
#define LSR_ERRS  (LSR_OE | LSR_PE | LSR_FE | LSR_BI)

#define MSR_DCTS  0x01
#define MSR_DDSR  0x02
#define MSR_DDCD  0x08
#define MSR_CTS   0x10
#define MSR_DSR   0x20
#define MSR_RI    0x40
#define MSR_DCD   0x80
#define MSR_LINES (MSR_CTS | MSR_DSR | MSR_DCD)

#define MCR_DTR   0x01
#define MCR_RTS   0x02

#define LCR_8N1   0x03
#define LCR_7E1   0x1A
#define LCR_5N1   0x00

#define FCR_OFF        0x00
#define FCR_ON_CLEAR   0x07  /* enable, clear RX/TX, 1-byte trigger */

#define BYTE_TIMEOUT_SECS    3
#define CONNECT_TIMEOUT_SECS 15
#define DROP_TIMEOUT_SECS    60
#define STREAM_BYTES         1024
#define STREAM_TIMEOUT_SECS  30
#define DROP_ECHO_BYTES      64

static unsigned int com1 = 0;

static const unsigned int baud_divs[] = { 96, 48, 12, 6, 3, 2, 1 };
static const long baud_rates[] = { 1200L, 2400L, 9600L, 19200L, 38400L, 57600L, 115200L };
#define NUM_BAUDS (sizeof(baud_divs) / sizeof(baud_divs[0]))

static unsigned int get_com1_base(void)
{
    unsigned int far *bda = (unsigned int far *) MK_FP(0x0040, 0x0000);
    return *bda ? *bda : 0x3F8;
}

static void busy_wait_ms(long ms)
{
    clock_t start = clock();
    while ((clock() - start) < (clock_t) ((ms * CLOCKS_PER_SEC) / 1000L)) {
    }
}

static int expired(clock_t start, long secs)
{
    return (clock() - start) > (clock_t) (secs * CLOCKS_PER_SEC);
}

static void uart_setup(unsigned int div, unsigned char lcr, unsigned char fcr)
{
    outp(com1 + REG_IER, 0x00);
    outp(com1 + REG_LCR, 0x80);
    outp(com1 + REG_DLL, div & 0xFF);
    outp(com1 + REG_DLH, (div >> 8) & 0xFF);
    outp(com1 + REG_LCR, lcr);
    outp(com1 + REG_FCR, fcr);
    outp(com1 + REG_MCR, MCR_DTR | MCR_RTS);
}

/* Discard anything in flight until the line is quiet for quiet_ms. */
static void drain(long quiet_ms)
{
    clock_t last = clock();
    while ((clock() - last) < (clock_t) ((quiet_ms * CLOCKS_PER_SEC) / 1000L)) {
        if (inp(com1 + REG_LSR) & LSR_DR) {
            (void) inp(com1 + REG_DATA);
            last = clock();
        }
    }
    (void) inp(com1 + REG_LSR);
}

static int send_byte(unsigned char b)
{
    clock_t start = clock();
    while (!(inp(com1 + REG_LSR) & LSR_THRE)) {
        if (expired(start, BYTE_TIMEOUT_SECS)) {
            return -1;
        }
    }
    outp(com1 + REG_DATA, b);
    return 0;
}

static int recv_byte(unsigned char *out, unsigned char *lsr_errs)
{
    clock_t start = clock();
    unsigned char lsr;
    while (!((lsr = (unsigned char) inp(com1 + REG_LSR)) & LSR_DR)) {
        *lsr_errs |= lsr & LSR_ERRS;
        if (expired(start, BYTE_TIMEOUT_SECS)) {
            return -1;
        }
    }
    *lsr_errs |= lsr & LSR_ERRS;
    *out = (unsigned char) inp(com1 + REG_DATA);
    return 0;
}

/* Read the MSR twice so the second read has no stale delta bits. */
static unsigned char msr_settled(void)
{
    (void) inp(com1 + REG_MSR);
    busy_wait_ms(20);
    return (unsigned char) inp(com1 + REG_MSR);
}

static int wait_connected(long secs)
{
    clock_t start = clock();
    while ((inp(com1 + REG_MSR) & MSR_LINES) != MSR_LINES) {
        if (expired(start, secs)) {
            return -1;
        }
    }
    return 0;
}

/* Send one byte and read its echo. Returns 0 on match, 1 on mismatch,
 * -1 on timeout. *got receives what came back. */
static int echo_byte(unsigned char b, unsigned char *got, unsigned char *lsr_errs)
{
    if (send_byte(b) != 0) {
        return -1;
    }
    if (recv_byte(got, lsr_errs) != 0) {
        return -1;
    }
    return (*got == b) ? 0 : 1;
}

static int do_status(void)
{
    static const unsigned char combos[] = { 0x00, MCR_DTR, MCR_RTS, MCR_DTR | MCR_RTS };
    unsigned char msr;
    unsigned int i;
    int fail = 0;

    uart_setup(12, LCR_8N1, FCR_OFF);
    if (wait_connected(CONNECT_TIMEOUT_SECS) != 0) {
        printf("STATUS FAIL NOT CONNECTED msr=%02X\n", inp(com1 + REG_MSR));
        return 1;
    }
    for (i = 0; i < sizeof(combos); i++) {
        outp(com1 + REG_MCR, combos[i]);
        busy_wait_ms(100);
        msr = msr_settled();
        printf("  mcr=%02X msr=%02X\n", combos[i], msr);
        if ((msr & MSR_LINES) != MSR_LINES || (msr & MSR_RI)) {
            fail = 1;
        }
    }
    outp(com1 + REG_MCR, MCR_DTR | MCR_RTS);
    if (fail) {
        printf("STATUS FAIL lines moved with DTR/RTS or RI set\n");
        return 1;
    }
    printf("STATUS OK\n");
    return 0;
}

static int do_binary(void)
{
    unsigned int b, n;
    unsigned char got, errs;
    int r, fail = 0;

    for (n = 0; n < NUM_BAUDS; n++) {
        uart_setup(baud_divs[n], LCR_8N1, FCR_OFF);
        drain(300);
        errs = 0;
        for (b = 0; b < 256; b++) {
            r = echo_byte((unsigned char) b, &got, &errs);
            if (r != 0) {
                break;
            }
        }
        if (r < 0) {
            printf("  %6ld TIMEOUT at byte %02X\n", baud_rates[n], b);
            fail = 1;
        } else if (r > 0) {
            printf("  %6ld MISMATCH sent=%02X got=%02X\n", baud_rates[n], b, got);
            fail = 1;
        } else if (errs) {
            printf("  %6ld LSR errors=%02X\n", baud_rates[n], errs);
            fail = 1;
        } else {
            printf("  %6ld 256/256\n", baud_rates[n]);
        }
    }
    if (fail) {
        printf("BINARY FAIL\n");
        return 1;
    }
    printf("BINARY OK\n");
    return 0;
}

static int stream_once(unsigned int div, unsigned char fcr, const char *label)
{
    unsigned int sent = 0, recvd = 0, bad = 0;
    unsigned char lsr, errs = 0, got, want;
    clock_t start;

    uart_setup(div, LCR_8N1, fcr);
    drain(300);
    start = clock();
    while (recvd < STREAM_BYTES) {
        lsr = (unsigned char) inp(com1 + REG_LSR);
        errs |= lsr & LSR_ERRS;
        if (lsr & LSR_DR) {
            got = (unsigned char) inp(com1 + REG_DATA);
            want = (unsigned char) ((recvd * 7 + 3) & 0xFF);
            if (got != want) {
                bad++;
            }
            recvd++;
        } else if (sent < STREAM_BYTES && (lsr & LSR_THRE)) {
            outp(com1 + REG_DATA, (unsigned char) ((sent * 7 + 3) & 0xFF));
            sent++;
        }
        if (expired(start, STREAM_TIMEOUT_SECS)) {
            break;
        }
    }
    outp(com1 + REG_FCR, FCR_OFF);
    if (recvd < STREAM_BYTES || bad || errs) {
        printf("  %s sent=%u recvd=%u bad=%u lsr=%02X\n", label, sent, recvd, bad, errs);
        return 1;
    }
    printf("  %s %u/%u\n", label, recvd, STREAM_BYTES);
    return 0;
}

static int do_stream(void)
{
    int fail = 0;
    fail |= stream_once(1, FCR_ON_CLEAR, "115200 fifo");
    fail |= stream_once(12, FCR_OFF, "  9600 nofifo");
    if (fail) {
        printf("STREAM FAIL\n");
        return 1;
    }
    printf("STREAM OK\n");
    return 0;
}

static int wordlen_case(unsigned char lcr, unsigned char sent, unsigned char want, const char *label)
{
    unsigned char got = 0, errs = 0;
    int r;

    uart_setup(12, lcr, FCR_OFF);
    drain(300);
    r = echo_byte(sent, &got, &errs);
    if (r < 0) {
        printf("  %s TIMEOUT\n", label);
        return 1;
    }
    if (got != want || (errs & (LSR_PE | LSR_FE))) {
        printf("  %s sent=%02X got=%02X want=%02X lsr=%02X\n", label, sent, got, want, errs);
        return 1;
    }
    printf("  %s %02X->%02X\n", label, sent, got);
    return 0;
}

static int do_wordlen(void)
{
    int fail = 0;
    fail |= wordlen_case(LCR_7E1, 0xC1, 0x41, "7E1");
    fail |= wordlen_case(LCR_7E1, 0x41, 0x41, "7E1");
    fail |= wordlen_case(LCR_5N1, 0xFF, 0x1F, "5N1");
    uart_setup(12, LCR_8N1, FCR_OFF);
    if (fail) {
        printf("WORDLEN FAIL\n");
        return 1;
    }
    printf("WORDLEN OK\n");
    return 0;
}

static int do_drop(void)
{
    unsigned char msr = 0, seen_delta = 0, got, errs = 0;
    unsigned int i;
    clock_t start;

    uart_setup(12, LCR_8N1, FCR_OFF);
    if (wait_connected(CONNECT_TIMEOUT_SECS) != 0) {
        printf("DROP FAIL NOT CONNECTED msr=%02X\n", inp(com1 + REG_MSR));
        return 1;
    }
    (void) inp(com1 + REG_MSR);
    printf("DROP READY\n");

    /* MSR reads clear the delta bits, so one read must see both. */
    start = clock();
    for (;;) {
        msr = (unsigned char) inp(com1 + REG_MSR);
        seen_delta |= msr & (MSR_DCTS | MSR_DDSR | MSR_DDCD);
        if (!(msr & MSR_DCD)) {
            break;
        }
        if (expired(start, DROP_TIMEOUT_SECS)) {
            printf("DROP FAIL DCD never dropped msr=%02X\n", msr);
            return 1;
        }
    }
    if (msr & (MSR_CTS | MSR_DSR)) {
        printf("DROP FAIL DCD low but CTS/DSR still set msr=%02X\n", msr);
        return 1;
    }
    if ((seen_delta & (MSR_DCTS | MSR_DDSR | MSR_DDCD)) != (MSR_DCTS | MSR_DDSR | MSR_DDCD)) {
        printf("DROP FAIL delta bits not latched deltas=%02X\n", seen_delta);
        return 1;
    }
    printf("DROP SEEN msr=%02X\n", msr);

    if (wait_connected(DROP_TIMEOUT_SECS) != 0) {
        printf("DROP FAIL no reconnect msr=%02X\n", inp(com1 + REG_MSR));
        return 1;
    }
    printf("RECONNECT SEEN msr=%02X\n", msr_settled());

    drain(300);
    for (i = 0; i < DROP_ECHO_BYTES; i++) {
        if (echo_byte((unsigned char) (0x30 + i), &got, &errs) != 0) {
            printf("DROP FAIL echo after reconnect at %u got=%02X\n", i, got);
            return 1;
        }
    }
    printf("DROP OK\n");
    return 0;
}

int main(int argc, char *argv[])
{
    int fail = 0;
    const char *mode = (argc > 1) ? argv[1] : "all";

    com1 = get_com1_base();
    printf("FNTEST %s com1=%03X\n", mode, com1);

    if (stricmp(mode, "status") == 0) {
        fail = do_status();
    } else if (stricmp(mode, "binary") == 0) {
        fail = do_binary();
    } else if (stricmp(mode, "stream") == 0) {
        fail = do_stream();
    } else if (stricmp(mode, "wordlen") == 0) {
        fail = do_wordlen();
    } else if (stricmp(mode, "drop") == 0) {
        fail = do_drop();
    } else if (stricmp(mode, "all") == 0) {
        fail |= do_status();
        fail |= do_binary();
        fail |= do_stream();
        fail |= do_wordlen();
        printf(fail ? "FNTEST ALL FAIL\n" : "FNTEST ALL OK\n");
    } else {
        printf("usage: fntest status|binary|stream|wordlen|all|drop\n");
        return 2;
    }
    return fail ? 1 : 0;
}
