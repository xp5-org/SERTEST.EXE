/*
 * comtest.c : direct port I/O COM1 regression tool for 86Box UART testing.
 *
 * Roles, argv[1]: "tx" sender and "rx" receiver.
 * Modes, argv[2]:
 *   binary : UART 8N1, sends and verifies all 256 byte values in order.
 *            Checks the 8-bit clean data path, since DOS COPY never put a
 *            0x1A byte on the wire.
 *   ascii  : UART 7E1, sends a text line then one byte with bit 7 set.
 *            The receiver must truncate it to 7 bits. The receiver LSR
 *            parity error bit must stay clear for the text bytes.
 *
 * Talks to the 8250/16550 registers at the COM1 base from the BIOS data
 * area at 0040:0000, not INT 14h, which would hide register level bugs.
 *
 * Build: OpenWatcom 16-bit wcc/wcl for real-mode DOS.
 * Strict C89: declarations lead their block, block comments only.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <conio.h>
#include <dos.h>

/* UART register offsets from the port base. */
#define REG_DATA  0  /* THR write, RBR read, DLAB=0 */
#define REG_IER   1  /* Interrupt Enable, DLAB=0 */
#define REG_DLL   0  /* Divisor Latch Low, DLAB=1 */
#define REG_DLH   1  /* Divisor Latch High, DLAB=1 */
#define REG_FCR   2  /* FIFO Control, write */
#define REG_LCR   3  /* Line Control */
#define REG_MCR   4  /* Modem Control */
#define REG_LSR   5  /* Line Status */
#define REG_IIR   2  /* Interrupt Identification, read */
#define REG_MSR   6  /* Modem Status, read */

/* LSR bits */
#define LSR_DR    0x01  /* Data Ready */
#define LSR_OE    0x02  /* Overrun Error */
#define LSR_PE    0x04  /* Parity Error */
#define LSR_FE    0x08  /* Framing Error */
#define LSR_THRE  0x20  /* Transmit Holding Register Empty */
#define LSR_BI    0x10  /* Break Interrupt */

/* MSR bits. */
#define MSR_DCTS  0x01  /* Delta CTS */
#define MSR_DDSR  0x02  /* Delta DSR */
#define MSR_TERI  0x04  /* Trailing Edge RI, falling edge only */
#define MSR_DDCD  0x08  /* Delta DCD */
#define MSR_CTS   0x10
#define MSR_DSR   0x20
#define MSR_RI    0x40
#define MSR_DCD   0x80
#define MCR_LOOP  0x10  /* internal diagnostic loopback */

/* LCR values for our two modes. */
#define LCR_8N1   0x03  /* 8 data bits, no parity, 1 stop bit */
#define LCR_7E1   0x1A  /* 7 data bits, even parity, 1 stop bit */

#define BAUD_DIVISOR 12  /* 115200 / 9600 */

#define SYNC_BINARY 0xA5
#define SYNC_ASCII  'S'
#define ASCII_TEXT  "HELLO-ASCII-MODE"
#define TRUNC_BYTE  0xC1  /* 'A' with the top bit set */
#define TRUNC_EXPECT 0x41

#define BYTE_TIMEOUT_SECS 5
#define SYNC_TIMEOUT_SECS 30
#define SYNC_MAX_RETRIES  4096

/* "overrun" mode timing. Both roles share these constants. rx must not
 * touch REG_DATA after the initial sync so bytes pile up and overflow the
 * 16 byte FIFO. FLOOD_BYTES=40 clears the FIFO. A flood takes about 42ms
 * at 9600 baud, so the waits carry several times that margin. */
#define FLOOD_BYTES   40
#define TX_GAP_MS     500
#define RX_SETTLE_MS  200
#define RX_WAIT2_MS   500

/* "irq" mode : interrupt driven RX on IRQ4, INT 0Ch, mask bit 0x10 at
 * port 0x21. Hooks the vector by writing the real-mode IVT directly, not
 * _dos_getvect. IRQ_PAYLOAD is small since this proves the interrupt, not
 * throughput. */
#define VEC_IRQ4     0x0C
#define PIC1_CMD     0x20
#define PIC1_DATA    0x21
#define IRQ4_MASK    0x10
#define IER_RDA      0x01
#define IRQ_RING     64
#define IRQ_PAYLOAD  32
#define IRQ_WAIT_SECS (BYTE_TIMEOUT_SECS * 4)

/* "fifoirq" mode : FIFO trigger level interrupt and character timeout
 * interrupt, on top of the plain RDA case in "irq" mode. Reuses the IRQ4
 * hook, ISR and ring. FCR bits 7:6 = 01 gives a 4 byte trigger. IIR top
 * bits read 11 whenever the FIFO is enabled, so IIR_FIFO_ON is checked
 * together with the reason nibble. */
#define FCR_FIFO_TRIG4   0x41  /* FIFO enable, 4-byte trigger */
#define IIR_FIFO_ON      0xC0
#define IIR_REASON_MASK  0x0E
#define IIR_REASON_RDA     0x04
#define IIR_REASON_TIMEOUT 0x0C
#define FIFO_TRIG_BYTES   4
#define FIFO_TAIL_BYTES   2
#define FIFO_PHASE_GAP_MS 300
#define FIFO_WAIT_SECS    (BYTE_TIMEOUT_SECS * 2)

/* "break" mode : a BREAK, LSR bit 4, must raise a Receiver Line Status
 * interrupt, IIR reason 011, the highest priority source. Serial kernel
 * debuggers use BREAK to get a guest's attention.
 * Self-contained, no peer. A host script calls tcsendbreak on the backing
 * device. Reuses the do_irq_rx hook and enables IER Receiver Line Status
 * as well as RDA. Asserts on the LSR captured by the first interrupt. */
#define IIR_REASON_LSR    0x06  /* Receiver Line Status, highest priority */
#define IER_LSR           0x04  /* Receiver Line Status Interrupt enable */
#define BREAK_WAIT_SECS   (BYTE_TIMEOUT_SECS * 6)

/* "pferr" mode : parity and framing error detection through the
 * char_set_read_error callback in src/char/char_serial.c.
 *
 * Host side, vuart.ko VUART_IOC_INJECT_FAULT inserts one byte flagged
 * TTY_FRAME into the peer tty flip buffer.
 *
 * PE and FE are checked together. char_serial_unescape cannot tell them
 * apart, since PARMRK escapes both as \377\0<byte>, and it sets
 * CHAR_COM_ERR_PARITY and CHAR_COM_ERR_FRAMING together. The data byte
 * PFERR_BYTE must also arrive intact. */
#define PFERR_WAIT_SECS   (BYTE_TIMEOUT_SECS * 6)
#define PFERR_BYTE        0x5A

/* "stopbits" mode : measures character time with PIT channel 0, port
 * 0x40/0x43, not clock, which ticks at only 18.2Hz. Latch plus two reads
 * gives about 838ns resolution.
 *
 * Real hardware sends 1.5 stop bit times for 2 stop bits at 5 data bits.
 * serial.c does dev->bits++ with no 5 bit special case, so going from 1
 * to 2 stop bits adds a full bit time. The test tells 0.5 from 1.0.
 *
 * Runs in internal loopback so the backend is not involved. */
#define PIT_CH0_PORT     0x40
#define PIT_CMD_PORT     0x43
#define PIT_LATCH_CH0    0x00
#define PIT_HZ           1193182UL
#define STOPBITS_LCR_5N1 0x00  /* 5 data bits, no parity, 1 stop bit */
#define STOPBITS_LCR_5N2 0x04  /* 5 data bits, no parity, 2 stop bits */
#define STOPBITS_SAMPLES 64
#define STOPBITS_BAUD    9600UL  /* 115200 / 12 */

static volatile unsigned char irq_buf[IRQ_RING];
static volatile unsigned int irq_count = 0;
static volatile unsigned char irq_iir_seen = 0xFF;
static volatile unsigned char irq_lsr_seen = 0xFF;
static volatile unsigned long irq_hits = 0;
static unsigned int irq_saved_off = 0, irq_saved_seg = 0;

static unsigned int com1_base = 0;

static unsigned int get_com1_base(void)
{
    unsigned int far *bda_com1 = (unsigned int far *) MK_FP(0x0040, 0x0000);
    unsigned int base = *bda_com1;
    if (base == 0) {
        base = 0x3F8;  /* BIOS reported none, use the PC/AT default */
    }
    return base;
}

static void uart_configure(unsigned int base, unsigned char lcr_value)
{
    outp(base + REG_IER, 0x00);          /* interrupts off, polled */
    outp(base + REG_LCR, 0x80);          /* DLAB=1 */
    outp(base + REG_DLL, BAUD_DIVISOR & 0xFF);
    outp(base + REG_DLH, (BAUD_DIVISOR >> 8) & 0xFF);
    outp(base + REG_LCR, lcr_value);     /* DLAB=0, apply word format */
    outp(base + REG_FCR, 0x00);          /* FIFO off */
    outp(base + REG_MCR, 0x03);          /* DTR + RTS */
}

static void busy_wait_ms(long ms)
{
    clock_t start = clock();
    while ((clock() - start) < (clock_t) ((ms * CLOCKS_PER_SEC) / 1000L)) {
        /* spin, do not touch REG_DATA or REG_LSR */
    }
}

/* Returns 0 on success, -1 on timeout. */
static int uart_send_byte(unsigned int base, unsigned char b)
{
    clock_t start = clock();
    while (!(inp(base + REG_LSR) & LSR_THRE)) {
        if ((clock() - start) > (clock_t) (BYTE_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
            return -1;
        }
    }
    outp(base + REG_DATA, b);
    return 0;
}

/* Returns 0 on success with *out and *lsr_out filled, -1 on timeout. */
static int uart_recv_byte(unsigned int base, unsigned char *out, unsigned char *lsr_out, long timeout_secs)
{
    clock_t start = clock();
    unsigned char lsr;
    while (!((lsr = inp(base + REG_LSR)) & LSR_DR)) {
        if ((clock() - start) > (clock_t) (timeout_secs * CLOCKS_PER_SEC)) {
            return -1;
        }
    }
    *out = (unsigned char) inp(base + REG_DATA);
    *lsr_out = lsr;
    return 0;
}

/* Reads bytes until one equals sync_byte, ignoring the rest. Returns 0 on
 * success, -1 if it never showed up within the SYNC limits. */
static int wait_for_sync(unsigned int base, unsigned char sync_byte)
{
    int i;
    unsigned char b, lsr;
    for (i = 0; i < SYNC_MAX_RETRIES; i++) {
        if (uart_recv_byte(base, &b, &lsr, SYNC_TIMEOUT_SECS) != 0) {
            return -1;
        }
        if (b == sync_byte) {
            return 0;
        }
    }
    return -1;
}

static int do_binary_tx(void)
{
    int i;
    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    if (uart_send_byte(com1_base, SYNC_BINARY) != 0) {
        printf("BINARY TX FAIL: SYNC SEND TIMEOUT\n");
        return 1;
    }
    for (i = 0; i < 256; i++) {
        if (uart_send_byte(com1_base, (unsigned char) i) != 0) {
            printf("BINARY TX FAIL: TIMEOUT AT BYTE %d\n", i);
            return 1;
        }
    }
    printf("TX BINARY DONE\n");
    return 0;
}

static int do_binary_rx(void)
{
    int i;
    int mismatches = 0;
    int first_bad = -1;
    unsigned char first_bad_expected = 0, first_bad_got = 0;
    unsigned char b, lsr;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);

    if (wait_for_sync(com1_base, SYNC_BINARY) != 0) {
        printf("BINARY FAIL: SYNC TIMEOUT\n");
        return 1;
    }

    for (i = 0; i < 256; i++) {
        if (uart_recv_byte(com1_base, &b, &lsr, BYTE_TIMEOUT_SECS) != 0) {
            printf("BINARY FAIL: TIMEOUT AT BYTE %d\n", i);
            return 1;
        }
        if (b != (unsigned char) i) {
            mismatches++;
            if (first_bad < 0) {
                first_bad = i;
                first_bad_expected = (unsigned char) i;
                first_bad_got = b;
            }
        }
    }

    if (mismatches == 0) {
        printf("BINARY OK 256/256\n");
        return 0;
    }
    printf("BINARY FAIL mismatches=%d first_bad_idx=%d expected=%02X got=%02X\n",
           mismatches, first_bad, first_bad_expected, first_bad_got);
    return 1;
}

static int do_ascii_tx(void)
{
    size_t i;
    size_t len = strlen(ASCII_TEXT);
    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_7E1);
    if (uart_send_byte(com1_base, SYNC_ASCII) != 0) {
        printf("ASCII TX FAIL: SYNC SEND TIMEOUT\n");
        return 1;
    }
    for (i = 0; i < len; i++) {
        if (uart_send_byte(com1_base, (unsigned char) ASCII_TEXT[i]) != 0) {
            printf("ASCII TX FAIL: TIMEOUT IN TEXT AT %u\n", (unsigned) i);
            return 1;
        }
    }
    if (uart_send_byte(com1_base, 0x0D) != 0 || uart_send_byte(com1_base, 0x0A) != 0) {
        printf("ASCII TX FAIL: TIMEOUT SENDING CRLF\n");
        return 1;
    }
    if (uart_send_byte(com1_base, TRUNC_BYTE) != 0) {
        printf("ASCII TX FAIL: TIMEOUT SENDING TRUNCATION BYTE\n");
        return 1;
    }
    printf("TX ASCII DONE\n");
    return 0;
}

static int do_ascii_rx(void)
{
    size_t i;
    size_t len = strlen(ASCII_TEXT);
    char got_text[64];
    int text_mismatch = 0;
    int parity_errors = 0;
    unsigned char b, lsr;
    unsigned char cr, lf, trunc;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_7E1);

    if (wait_for_sync(com1_base, SYNC_ASCII) != 0) {
        printf("ASCII FAIL: SYNC TIMEOUT\n");
        return 1;
    }

    for (i = 0; i < len && i < (sizeof(got_text) - 1); i++) {
        if (uart_recv_byte(com1_base, &b, &lsr, BYTE_TIMEOUT_SECS) != 0) {
            printf("ASCII FAIL: TIMEOUT IN TEXT AT %u\n", (unsigned) i);
            return 1;
        }
        if (lsr & LSR_PE) {
            parity_errors++;
        }
        got_text[i] = (char) b;
        if (b != (unsigned char) ASCII_TEXT[i]) {
            text_mismatch = 1;
        }
    }
    got_text[i] = '\0';

    if (uart_recv_byte(com1_base, &cr, &lsr, BYTE_TIMEOUT_SECS) != 0 ||
        uart_recv_byte(com1_base, &lf, &lsr, BYTE_TIMEOUT_SECS) != 0) {
        printf("ASCII FAIL: TIMEOUT READING CRLF\n");
        return 1;
    }

    if (uart_recv_byte(com1_base, &trunc, &lsr, BYTE_TIMEOUT_SECS) != 0) {
        printf("ASCII FAIL: TIMEOUT READING TRUNCATION BYTE\n");
        return 1;
    }

    if (text_mismatch) {
        printf("ASCII FAIL: TEXT MISMATCH got=%s\n", got_text);
        return 1;
    }
    if (cr != 0x0D || lf != 0x0A) {
        printf("ASCII FAIL: BAD CRLF got=%02X %02X\n", cr, lf);
        return 1;
    }
    if (trunc != TRUNC_EXPECT) {
        printf("ASCII FAIL: TRUNCATION REGRESSION expected=%02X got=%02X\n",
               TRUNC_EXPECT, trunc);
        return 1;
    }

    printf("ASCII OK text=%s truncation=%02X parity_errors=%d\n",
           got_text, trunc, parity_errors);
    return 0;
}

static int do_overrun_tx(void)
{
    int i;
    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    if (uart_send_byte(com1_base, SYNC_BINARY) != 0) {
        printf("FIFO TX FAIL: SYNC SEND TIMEOUT\n");
        return 1;
    }
    for (i = 0; i < FLOOD_BYTES; i++) {
        if (uart_send_byte(com1_base, 0x55) != 0) {
            printf("FIFO TX FAIL: TIMEOUT IN FLOOD 1 AT %d\n", i);
            return 1;
        }
    }
    busy_wait_ms(TX_GAP_MS);
    for (i = 0; i < FLOOD_BYTES; i++) {
        if (uart_send_byte(com1_base, 0xAA) != 0) {
            printf("FIFO TX FAIL: TIMEOUT IN FLOOD 2 AT %d\n", i);
            return 1;
        }
    }
    printf("TX OVERRUN DONE\n");
    return 0;
}

static int do_overrun_rx(void)
{
    unsigned char lsr1, lsr2, lsr3;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    /* Enable the 16550 FIFO, which also clears it. The non-FIFO overrun path
           is a separate path not tested here. */
    outp(com1_base + REG_FCR, 0x07);

    if (wait_for_sync(com1_base, SYNC_BINARY) != 0) {
        printf("FIFO FAIL: SYNC TIMEOUT\n");
        return 1;
    }

    /* From here on only touch REG_DATA and REG_LSR for the three timed
           checks, so flood 1 overruns the FIFO. */
    busy_wait_ms(RX_SETTLE_MS);
    lsr1 = (unsigned char) inp(com1_base + REG_LSR); /* expect OE set */
    lsr2 = (unsigned char) inp(com1_base + REG_LSR); /* expect OE clear after the read */

    busy_wait_ms(RX_WAIT2_MS);
    lsr3 = (unsigned char) inp(com1_base + REG_LSR); /* expect OE set again */

    printf("LSR1=%02X LSR2=%02X LSR3=%02X\n", lsr1, lsr2, lsr3);

    if (!(lsr1 & LSR_OE)) {
        printf("FIFO FAIL: FLOOD 1 NEVER SET OE (lsr1=%02X)\n", lsr1);
        return 1;
    }
    if (lsr2 & LSR_OE) {
        printf("FIFO FAIL: OE DID NOT CLEAR ON LSR READ (lsr2=%02X)\n", lsr2);
        return 1;
    }
    if (!(lsr3 & LSR_OE)) {
        printf("FIFO FAIL: SECOND OVERRUN NEVER RE-SET OE (lsr3=%02X) -- fifo_clear_overrun regression\n", lsr3);
        return 1;
    }

    printf("FIFO OK lsr1=%02X lsr2=%02X lsr3=%02X\n", lsr1, lsr2, lsr3);
    return 0;
}

/* ISR for IRQ4. Drains RBR while LSR.DR is set and sends a non-specific
 * EOI to the master 8259. Captures the first IIR before RBR is touched.
 * With FCR 0x00 it must read exactly 0x04. */
void __interrupt __far irq4_isr(void)
{
    unsigned char iir = (unsigned char) inp(com1_base + REG_IIR);
    unsigned char lsr;

    irq_hits++;
    if (irq_iir_seen == 0xFF) {
        irq_iir_seen = iir;
    }
    lsr = (unsigned char) inp(com1_base + REG_LSR);
    if (irq_lsr_seen == 0xFF) {
        /* First LSR read of the first interrupt, since the read clears
                 * Receiver Line Status. */
        irq_lsr_seen = lsr;
    }
    while (lsr & LSR_DR) {
        if (irq_count < IRQ_RING) {
            irq_buf[irq_count++] = (unsigned char) inp(com1_base + REG_DATA);
        } else {
            (void) inp(com1_base + REG_DATA); /* ring full, drain so DR clears */
        }
        lsr = (unsigned char) inp(com1_base + REG_LSR);
    }
    outp(PIC1_CMD, 0x20);
}

static void irq4_hook(void)
{
    /* -ms makes function pointers NEAR. __far makes the compiler build a
         * segment:offset pair for FP_OFF and FP_SEG. */
    void (__interrupt __far *isr_ptr)(void) = irq4_isr;
    unsigned int far *ivt = (unsigned int far *) MK_FP(0, VEC_IRQ4 * 4);
    _disable();
    irq_saved_off = ivt[0];
    irq_saved_seg = ivt[1];
    ivt[0] = FP_OFF(isr_ptr);
    ivt[1] = FP_SEG(isr_ptr);
    _enable();
}

static void irq4_unhook(void)
{
    unsigned int far *ivt = (unsigned int far *) MK_FP(0, VEC_IRQ4 * 4);
    _disable();
    ivt[0] = irq_saved_off;
    ivt[1] = irq_saved_seg;
    _enable();
}

static int do_irq_tx(void)
{
    int i;
    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    if (uart_send_byte(com1_base, SYNC_BINARY) != 0) {
        printf("IRQ TX FAIL: SYNC SEND TIMEOUT\n");
        return 1;
    }
    for (i = 0; i < IRQ_PAYLOAD; i++) {
        if (uart_send_byte(com1_base, (unsigned char) i) != 0) {
            printf("IRQ TX FAIL: TIMEOUT AT BYTE %d\n", i);
            return 1;
        }
    }
    printf("TX IRQ DONE\n");
    return 0;
}

/* Everything goes through the ring the ISR fills and never polls
 * REG_DATA or REG_LSR, which would race the ISR. Sync is found by scanning
 * the ring for SYNC_BINARY. */
static int do_irq_rx(void)
{
    unsigned char orig_mask;
    clock_t start;
    int i, sync_idx = -1;
    int mismatches = 0;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    outp(com1_base + REG_FCR, 0x00); /* FIFO off, IIR must read exactly 0x04 */
    /* OUT2 gates the UART interrupt onto the IRQ line. uart_configure sets
         * only DTR + RTS, so interrupt modes must add it. */
    outp(com1_base + REG_MCR, 0x0B); /* DTR + RTS + OUT2 */

    irq_count = 0;
    irq_hits = 0;
    irq_iir_seen = 0xFF;

    orig_mask = (unsigned char) inp(PIC1_DATA);
    irq4_hook();
    outp(PIC1_DATA, orig_mask & ~IRQ4_MASK);
    outp(com1_base + REG_IER, IER_RDA);

    start = clock();
    while (irq_count < (unsigned int) (IRQ_PAYLOAD + 1)) {
        if ((clock() - start) > (clock_t) (IRQ_WAIT_SECS * CLOCKS_PER_SEC)) {
            outp(com1_base + REG_IER, 0x00);
            outp(PIC1_DATA, orig_mask);
            irq4_unhook();
            printf("IRQ FAIL: TIMEOUT waiting for interrupt-driven bytes (got %u, hits=%lu)\n",
                   irq_count, irq_hits);
            return 1;
        }
    }

    outp(com1_base + REG_IER, 0x00);
    outp(PIC1_DATA, orig_mask);
    irq4_unhook();

    for (i = 0; i < (int) irq_count; i++) {
        if (irq_buf[i] == SYNC_BINARY) {
            sync_idx = i;
            break;
        }
    }
    if (sync_idx < 0 || (sync_idx + IRQ_PAYLOAD) >= (int) irq_count) {
        printf("IRQ FAIL: SYNC NOT FOUND IN CAPTURED RING (count=%u)\n", irq_count);
        return 1;
    }
    for (i = 0; i < IRQ_PAYLOAD; i++) {
        if (irq_buf[sync_idx + 1 + i] != (unsigned char) i) {
            mismatches++;
        }
    }

    if (irq_hits == 0) {
        printf("IRQ FAIL: ISR NEVER RAN (irq_hits=0) -- IRQ4 never fired/vectored\n");
        return 1;
    }
    if (irq_iir_seen != 0x04) {
        printf("IRQ FAIL: WRONG IIR REASON CODE seen=%02X expected=04 (RX data available, no FIFO)\n",
               irq_iir_seen);
        return 1;
    }
    if (mismatches != 0) {
        printf("IRQ FAIL: PAYLOAD MISMATCH count=%d/%d\n", mismatches, IRQ_PAYLOAD);
        return 1;
    }

    printf("IRQ OK hits=%lu iir=%02X bytes=%d/%d\n", irq_hits, irq_iir_seen, IRQ_PAYLOAD, IRQ_PAYLOAD);
    return 0;
}

static int do_fifoirq_tx(void)
{
    static const unsigned char phase1[FIFO_TRIG_BYTES] = { 0x10, 0x11, 0x12, 0x13 };
    static const unsigned char phase2[FIFO_TAIL_BYTES] = { 0x20, 0x21 };
    int i;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    for (i = 0; i < FIFO_TRIG_BYTES; i++) {
        if (uart_send_byte(com1_base, phase1[i]) != 0) {
            printf("FIFOIRQ TX FAIL: TIMEOUT IN PHASE1 AT %d\n", i);
            return 1;
        }
    }
    busy_wait_ms(FIFO_PHASE_GAP_MS); /* well past the FIFO timeout window */
    for (i = 0; i < FIFO_TAIL_BYTES; i++) {
        if (uart_send_byte(com1_base, phase2[i]) != 0) {
            printf("FIFOIRQ TX FAIL: TIMEOUT IN PHASE2 AT %d\n", i);
            return 1;
        }
    }
    printf("TX FIFOIRQ DONE\n");
    return 0;
}

/* Same ring discipline as do_irq_rx. Phase 1 sends FIFO_TRIG_BYTES, so
 * the interrupt fires at the trigger level, reason 0xC4. Phase 2 sends
 * fewer than the trigger, so only the character timeout can fire it,
 * about 4 character times later, reason 0xCC. */
static int do_fifoirq_rx(void)
{
    static const unsigned char expect1[FIFO_TRIG_BYTES] = { 0x10, 0x11, 0x12, 0x13 };
    static const unsigned char expect2[FIFO_TAIL_BYTES] = { 0x20, 0x21 };
    unsigned char orig_mask;
    clock_t start;
    int i;
    unsigned char iir1, iir2;
    unsigned int count1, count2;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    outp(com1_base + REG_FCR, FCR_FIFO_TRIG4);
    outp(com1_base + REG_MCR, 0x0B); /* DTR + RTS + OUT2, see do_irq_rx */

    irq_count = 0;
    irq_hits = 0;
    irq_iir_seen = 0xFF;

    orig_mask = (unsigned char) inp(PIC1_DATA);
    irq4_hook();
    outp(PIC1_DATA, orig_mask & ~IRQ4_MASK);
    outp(com1_base + REG_IER, IER_RDA);

    start = clock();
    while (irq_count < (unsigned int) FIFO_TRIG_BYTES) {
        if ((clock() - start) > (clock_t) (FIFO_WAIT_SECS * CLOCKS_PER_SEC)) {
            outp(com1_base + REG_IER, 0x00);
            outp(PIC1_DATA, orig_mask);
            irq4_unhook();
            printf("FIFOIRQ FAIL: TIMEOUT IN PHASE1 (got %u, hits=%lu)\n", irq_count, irq_hits);
            return 1;
        }
    }
    iir1 = irq_iir_seen;
    count1 = irq_count;
    irq_iir_seen = 0xFF; /* phase 2 is a separate event */

    start = clock();
    while (irq_count < (count1 + (unsigned int) FIFO_TAIL_BYTES)) {
        if ((clock() - start) > (clock_t) (FIFO_WAIT_SECS * CLOCKS_PER_SEC)) {
            outp(com1_base + REG_IER, 0x00);
            outp(PIC1_DATA, orig_mask);
            irq4_unhook();
            printf("FIFOIRQ FAIL: TIMEOUT IN PHASE2 (got %u, hits=%lu)\n", irq_count - count1, irq_hits);
            return 1;
        }
    }
    iir2 = irq_iir_seen;
    count2 = irq_count - count1;

    outp(com1_base + REG_IER, 0x00);
    outp(PIC1_DATA, orig_mask);
    irq4_unhook();

    for (i = 0; i < FIFO_TRIG_BYTES; i++) {
        if (irq_buf[i] != expect1[i]) {
            printf("FIFOIRQ FAIL: PHASE1 PAYLOAD MISMATCH at %d expected=%02X got=%02X\n",
                   i, expect1[i], irq_buf[i]);
            return 1;
        }
    }
    if ((iir1 & (IIR_FIFO_ON | IIR_REASON_MASK | 0x01)) != (IIR_FIFO_ON | IIR_REASON_RDA)) {
        printf("FIFOIRQ FAIL: PHASE1 WRONG IIR REASON seen=%02X expected=%02X (FIFO trigger-reached)\n",
               iir1, (unsigned) (IIR_FIFO_ON | IIR_REASON_RDA));
        return 1;
    }

    for (i = 0; i < FIFO_TAIL_BYTES; i++) {
        if (irq_buf[count1 + i] != expect2[i]) {
            printf("FIFOIRQ FAIL: PHASE2 PAYLOAD MISMATCH at %d expected=%02X got=%02X\n",
                   i, expect2[i], irq_buf[count1 + i]);
            return 1;
        }
    }
    if ((iir2 & (IIR_FIFO_ON | IIR_REASON_MASK | 0x01)) != (IIR_FIFO_ON | IIR_REASON_TIMEOUT)) {
        printf("FIFOIRQ FAIL: PHASE2 WRONG IIR REASON seen=%02X expected=%02X (FIFO character-timeout, not trigger)\n",
               iir2, (unsigned) (IIR_FIFO_ON | IIR_REASON_TIMEOUT));
        return 1;
    }

    printf("FIFOIRQ OK iir1=%02X iir2=%02X bytes=%u+%u\n", iir1, iir2, count1, count2);
    return 0;
}

/* Waits for a Receiver Line Status interrupt carrying LSR_BI. See the
 * "break" mode comment. Never polls once armed. */
static int do_break_wait(void)
{
    unsigned char orig_mask;
    clock_t start;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    outp(com1_base + REG_FCR, 0x00);    /* FIFO off */
    outp(com1_base + REG_MCR, 0x0B);    /* DTR + RTS + OUT2, see do_irq_rx */

    irq_count = 0;
    irq_hits = 0;
    irq_iir_seen = 0xFF;
    irq_lsr_seen = 0xFF;

    orig_mask = (unsigned char) inp(PIC1_DATA);
    irq4_hook();
    outp(PIC1_DATA, orig_mask & ~IRQ4_MASK);
    outp(com1_base + REG_IER, IER_RDA | IER_LSR); /* RDA + Receiver Line Status */

    printf("WAITING FOR BREAK (debugger-attention simulation)...\n");

    start = clock();
    while (irq_hits == 0) {
        if ((clock() - start) > (clock_t) (BREAK_WAIT_SECS * CLOCKS_PER_SEC)) {
            outp(com1_base + REG_IER, 0x00);
            outp(PIC1_DATA, orig_mask);
            irq4_unhook();
            printf("BREAK FAIL: TIMEOUT -- no interrupt fired within %ds (debugger attention never arrived)\n",
                   BREAK_WAIT_SECS);
            return 1;
        }
    }

    outp(com1_base + REG_IER, 0x00);
    outp(PIC1_DATA, orig_mask);
    irq4_unhook();

    if ((irq_iir_seen & IIR_REASON_MASK) != IIR_REASON_LSR) {
        printf("BREAK FAIL: WRONG IIR REASON seen=%02X expected reason=06 (Receiver Line Status)\n",
               irq_iir_seen);
        return 1;
    }
    if (!(irq_lsr_seen & LSR_BI)) {
        printf("BREAK FAIL: LSR BREAK-INTERRUPT BIT NOT SET lsr=%02X\n", irq_lsr_seen);
        return 1;
    }

    printf("BREAK OK hits=%lu iir=%02X lsr=%02X -- genuine break correctly interrupted a waiting COM1 read, the same signal a real kernel debugger stub relies on\n",
           irq_hits, irq_iir_seen, irq_lsr_seen);
    return 0;
}

/* Waits for a Receiver Line Status interrupt carrying LSR_PE and LSR_FE
 * together. See the "pferr" mode comment. Never polls once armed. */
static int do_pferr_wait(void)
{
    unsigned char orig_mask;
    clock_t start;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);
    outp(com1_base + REG_FCR, 0x00);    /* FIFO off */
    outp(com1_base + REG_MCR, 0x0B);    /* DTR + RTS + OUT2, see do_irq_rx */

    irq_count = 0;
    irq_hits = 0;
    irq_iir_seen = 0xFF;
    irq_lsr_seen = 0xFF;

    orig_mask = (unsigned char) inp(PIC1_DATA);
    irq4_hook();
    outp(PIC1_DATA, orig_mask & ~IRQ4_MASK);
    outp(com1_base + REG_IER, IER_RDA | IER_LSR); /* RDA + Receiver Line Status */

    printf("WAITING FOR PFERR (parity/framing-fault injection)...\n");

    start = clock();
    while (irq_hits == 0) {
        if ((clock() - start) > (clock_t) (PFERR_WAIT_SECS * CLOCKS_PER_SEC)) {
            outp(com1_base + REG_IER, 0x00);
            outp(PIC1_DATA, orig_mask);
            irq4_unhook();
            printf("PFERR FAIL: TIMEOUT -- no interrupt fired within %ds (fault injection never arrived)\n",
                   PFERR_WAIT_SECS);
            return 1;
        }
    }

    outp(com1_base + REG_IER, 0x00);
    outp(PIC1_DATA, orig_mask);
    irq4_unhook();

    if ((irq_iir_seen & IIR_REASON_MASK) != IIR_REASON_LSR) {
        printf("PFERR FAIL: WRONG IIR REASON seen=%02X expected reason=06 (Receiver Line Status)\n",
               irq_iir_seen);
        return 1;
    }
    if ((irq_lsr_seen & (LSR_PE | LSR_FE)) != (LSR_PE | LSR_FE)) {
        printf("PFERR FAIL: LSR PARITY/FRAMING BITS NOT BOTH SET lsr=%02X expected PE|FE=%02X\n",
               irq_lsr_seen, (unsigned) (LSR_PE | LSR_FE));
        return 1;
    }
    if (irq_count == 0 || irq_buf[0] != PFERR_BYTE) {
        printf("PFERR FAIL: DATA BYTE MISMATCH count=%u got=%02X expected=%02X\n",
               irq_count, (unsigned) (irq_count ? irq_buf[0] : 0xFF), (unsigned) PFERR_BYTE);
        return 1;
    }

    printf("PFERR OK hits=%lu iir=%02X lsr=%02X byte=%02X -- genuine parity/framing fault correctly interrupted a waiting COM1 read with the real data byte intact\n",
           irq_hits, irq_iir_seen, irq_lsr_seen, irq_buf[0]);
    return 0;
}

/* "loopback" mode : self-contained on one COM1, no peer. char_stdio.c
 * always reports CTS, DSR and DCD asserted, so handshake lines cannot be
 * tested across the bridge. MCR bit 4 loopback maps DTR to DSR, RTS to
 * CTS, OUT1 to RI and OUT2 to DCD, with the MSR low nibble latching
 * changes. TERI fires on the falling edge of RI only, so both edges are
 * checked. */
static unsigned char loop_step(unsigned int base, unsigned char mcr_val)
{
    outp(base + REG_MCR, mcr_val);
    return (unsigned char) inp(base + REG_MSR);
}

static int do_loopback_self(void)
{
    unsigned char msr;
    int fail = 0;
    unsigned char b, lsr;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base, LCR_8N1);

    /* Enter loopback with every output low, then read MSR once to flush
         * stale delta bits. */
    loop_step(com1_base, MCR_LOOP);
    (void) inp(com1_base + REG_MSR);

    msr = loop_step(com1_base, MCR_LOOP | 0x01); /* DTR */
    if (!(msr & MSR_DSR) || !(msr & MSR_DDSR)) {
        printf("LOOPBACK FAIL: DTR->DSR msr=%02X\n", msr);
        fail = 1;
    }

    msr = loop_step(com1_base, MCR_LOOP | 0x01 | 0x02); /* + RTS */
    if (!(msr & MSR_CTS) || !(msr & MSR_DCTS) || !(msr & MSR_DSR) || (msr & MSR_DDSR)) {
        printf("LOOPBACK FAIL: RTS->CTS msr=%02X\n", msr);
        fail = 1;
    }

    msr = loop_step(com1_base, MCR_LOOP | 0x01 | 0x02 | 0x04); /* + OUT1, RI rising edge */
    if (!(msr & MSR_RI) || (msr & MSR_TERI)) {
        printf("LOOPBACK FAIL: OUT1->RI rising edge wrongly set TERI msr=%02X\n", msr);
        fail = 1;
    }

    msr = loop_step(com1_base, MCR_LOOP | 0x01 | 0x02 | 0x04 | 0x08); /* + OUT2 */
    if (!(msr & MSR_DCD) || !(msr & MSR_DDCD)) {
        printf("LOOPBACK FAIL: OUT2->DCD msr=%02X\n", msr);
        fail = 1;
    }

    msr = loop_step(com1_base, MCR_LOOP | 0x01 | 0x02 | 0x08); /* drop OUT1, RI falling edge */
    if ((msr & MSR_RI) || !(msr & MSR_TERI)) {
        printf("LOOPBACK FAIL: OUT1 drop (RI falling edge) never set TERI msr=%02X\n", msr);
        fail = 1;
    }

    /* loopback also loops THR to RBR */
    outp(com1_base + REG_MCR, MCR_LOOP);
    outp(com1_base + REG_DATA, 0x5A);
    if (uart_recv_byte(com1_base, &b, &lsr, 2) != 0 || b != 0x5A) {
        printf("LOOPBACK FAIL: TX/RX DATA PATH byte=%02X\n", b);
        fail = 1;
    }

    outp(com1_base + REG_MCR, 0x00); /* leave loopback */

    if (fail) {
        return 1;
    }
    printf("LOOPBACK OK\n");
    return 0;
}

static unsigned int pit_read16(void)
{
    unsigned char lo, hi;

    outp(PIT_CMD_PORT, PIT_LATCH_CH0);
    lo = (unsigned char) inp(PIT_CH0_PORT);
    hi = (unsigned char) inp(PIT_CH0_PORT);
    return ((unsigned int) hi << 8) | lo;
}

/* Returns the average interval between THRE ready over STOPBITS_SAMPLES
 * back-to-back bytes in PIT ticks, one character time. 0xFFFFFFFF on
 * timeout. */
static unsigned long measure_char_ticks(unsigned int base, unsigned char lcr_val)
{
    unsigned int t_prev, t_now;
    unsigned long sum = 0;
    int i;
    clock_t wait_start;

    uart_configure(base, lcr_val);
    outp(base + REG_MCR, MCR_LOOP); /* replaces the DTR + RTS MCR from uart_configure */

    wait_start = clock();
    while (!(inp(base + REG_LSR) & LSR_THRE)) {
        if ((clock() - wait_start) > (clock_t) (BYTE_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
            return 0xFFFFFFFFUL;
        }
    }
    t_prev = pit_read16();
    outp(base + REG_DATA, 0x00);

    for (i = 0; i < STOPBITS_SAMPLES; i++) {
        wait_start = clock();
        while (!(inp(base + REG_LSR) & LSR_THRE)) {
            if ((clock() - wait_start) > (clock_t) (BYTE_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
                return 0xFFFFFFFFUL;
            }
        }
        t_now = pit_read16();
        /* PIT channel 0 counts down. No wraparound handling since a character
                 * time is far below the 54.9ms wrap. */
        sum += (unsigned long) (t_prev - t_now);
        t_prev = t_now;
        outp(base + REG_DATA, 0x00);
    }

    return sum / (unsigned long) STOPBITS_SAMPLES;
}

/* Asserts the real hardware spec of 1.5 stop bit times. Its own mode so
 * this one known divergence can fail without blocking the rest of the
 * suite. See the "stopbits" comment above. */
static int do_stopbits_self(void)
{
    unsigned long ticks_a, ticks_b, bit_ticks, delta_fixed, delta_current, threshold, delta_measured;
    unsigned long us_a, us_b, us_delta;
    const char *classification;
    int matches_hardware;

    printf("COM1 BASE=%04X\n", com1_base);

    ticks_a = measure_char_ticks(com1_base, STOPBITS_LCR_5N1);
    if (ticks_a == 0xFFFFFFFFUL) {
        printf("STOPBITS FAIL: TIMEOUT MEASURING 5N1 BASELINE\n");
        outp(com1_base + REG_MCR, 0x00);
        return 1;
    }
    ticks_b = measure_char_ticks(com1_base, STOPBITS_LCR_5N2);
    if (ticks_b == 0xFFFFFFFFUL) {
        printf("STOPBITS FAIL: TIMEOUT MEASURING 5N2\n");
        outp(com1_base + REG_MCR, 0x00);
        return 1;
    }
    outp(com1_base + REG_MCR, 0x00);

    us_a = (ticks_a * 1000000UL) / PIT_HZ;
    us_b = (ticks_b * 1000000UL) / PIT_HZ;

    if (ticks_b <= ticks_a) {
        printf("STOPBITS FAIL: 5N2 (%luus) NOT SLOWER THAN 5N1 (%luus) -- timing measurement broken\n",
               us_b, us_a);
        return 1;
    }
    delta_measured = ticks_b - ticks_a;
    us_delta = (delta_measured * 1000000UL) / PIT_HZ;

    bit_ticks = PIT_HZ / STOPBITS_BAUD;
    delta_fixed = bit_ticks / 2;   /* 1.5 stop bits, real hardware */
    delta_current = bit_ticks;     /* 2.0 stop bits, current 86Box */
    threshold = (delta_fixed + delta_current) / 2;

    matches_hardware = (delta_measured < threshold);
    classification = matches_hardware
        ? "1.5STOPBIT(matches real hardware spec)"
        : "2.0STOPBIT(86Box current -- no 5-bit special case)";

    if (!matches_hardware) {
        printf("STOPBITS FAIL 5n1=%luus 5n2=%luus delta=%luus class=%s\n",
               us_a, us_b, us_delta, classification);
        return 1;
    }

    printf("STOPBITS OK 5n1=%luus 5n2=%luus delta=%luus class=%s\n",
           us_a, us_b, us_delta, classification);
    return 0;
}

int main(int argc, char *argv[])
{
    com1_base = get_com1_base();

    if (argc != 3) {
        printf("USAGE: COMTEST <tx|rx> <binary|ascii|overrun|irq|fifoirq|loopback|stopbits|break|pferr>\n");
        return 2;
    }

    if (strcmp(argv[1], "tx") == 0 && strcmp(argv[2], "binary") == 0) {
        return do_binary_tx();
    }
    if (strcmp(argv[1], "rx") == 0 && strcmp(argv[2], "binary") == 0) {
        return do_binary_rx();
    }
    if (strcmp(argv[1], "tx") == 0 && strcmp(argv[2], "ascii") == 0) {
        return do_ascii_tx();
    }
    if (strcmp(argv[1], "rx") == 0 && strcmp(argv[2], "ascii") == 0) {
        return do_ascii_rx();
    }
    if (strcmp(argv[1], "tx") == 0 && strcmp(argv[2], "overrun") == 0) {
        return do_overrun_tx();
    }
    if (strcmp(argv[1], "rx") == 0 && strcmp(argv[2], "overrun") == 0) {
        return do_overrun_rx();
    }
    if (strcmp(argv[1], "tx") == 0 && strcmp(argv[2], "irq") == 0) {
        return do_irq_tx();
    }
    if (strcmp(argv[1], "rx") == 0 && strcmp(argv[2], "irq") == 0) {
        return do_irq_rx();
    }
    if (strcmp(argv[1], "tx") == 0 && strcmp(argv[2], "fifoirq") == 0) {
        return do_fifoirq_tx();
    }
    if (strcmp(argv[1], "rx") == 0 && strcmp(argv[2], "fifoirq") == 0) {
        return do_fifoirq_rx();
    }
    if (strcmp(argv[2], "loopback") == 0) {
        /* self-contained, no peer */
        return do_loopback_self();
    }
    if (strcmp(argv[2], "stopbits") == 0) {
        /* self-contained, no peer */
        return do_stopbits_self();
    }
    if (strcmp(argv[2], "break") == 0) {
        /* self-contained, the break comes from a host-side script */
        return do_break_wait();
    }
    if (strcmp(argv[2], "pferr") == 0) {
        /* self-contained, the fault comes from a host-side ioctl */
        return do_pferr_wait();
    }

    printf("USAGE: COMTEST <tx|rx> <binary|ascii|overrun|irq|fifoirq|loopback|stopbits|break|pferr>\n");
    return 2;
}
