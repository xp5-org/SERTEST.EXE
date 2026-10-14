/*
 * xmtest.c : XMODEM/CRC file-transfer tool for emulator UART testing.
 *
 * Ward Christensen XMODEM, one DOS machine to another on COM1 at 9600 baud.
 *
 * Receiver sends 'C' repeatedly to request CRC mode.
 * Sender replies with SOH, blk#, ~blk#, 128 data bytes, CRC hi, CRC lo.
 * Receiver sends ACK 0x06 when framing and CRC are good, else NAK 0x15 to
 * get the same block again. A duplicate block is re-ACKed and not stored.
 * EOT 0x04 ends the transfer and is ACKed. CAN 0x18 aborts.
 *
 * The sender corrupts the CRC of CORRUPT_BLOCK on its first attempt only,
 * to exercise the NAK and retransmit path.
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

#define LSR_DR    0x01
#define LSR_THRE  0x20

#define LCR_8N1   0x03
#define BAUD_DIVISOR 12  /* 115200 / 9600 */

#define XM_SOH  0x01
#define XM_EOT  0x04
#define XM_ACK  0x06
#define XM_NAK  0x15
#define XM_CAN  0x18
#define XM_C    0x43  /* requests CRC mode */

#define BLOCK_SIZE    128
#define FILE_BLOCKS   400    /* 51200 bytes */
#define CORRUPT_BLOCK 2      /* bad CRC on first attempt only */
#define MAX_RETRIES   10
#define XM_BYTE_TIMEOUT_SECS 5
#define XM_SYNC_TIMEOUT_SECS 30

static unsigned int com1_base = 0;

static unsigned int get_com1_base(void)
{
    unsigned int far *bda_com1 = (unsigned int far *) MK_FP(0x0040, 0x0000);
    unsigned int base = *bda_com1;
    if (base == 0) {
        base = 0x3F8;
    }
    return base;
}

static void uart_configure(unsigned int base)
{
    outp(base + REG_IER, 0x00);
    outp(base + REG_LCR, 0x80);
    outp(base + REG_DLL, BAUD_DIVISOR & 0xFF);
    outp(base + REG_DLH, (BAUD_DIVISOR >> 8) & 0xFF);
    outp(base + REG_LCR, LCR_8N1);
    outp(base + REG_FCR, 0x00);
    outp(base + REG_MCR, 0x0B); /* DTR, RTS, OUT2 */
}

static int uart_send_byte(unsigned int base, unsigned char b)
{
    clock_t start = clock();
    while (!(inp(base + REG_LSR) & LSR_THRE)) {
        if ((clock() - start) > (clock_t) (XM_BYTE_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
            return -1;
        }
    }
    outp(base + REG_DATA, b);
    return 0;
}

static int uart_recv_byte(unsigned int base, unsigned char *out, long timeout_secs)
{
    clock_t start = clock();
    while (!(inp(base + REG_LSR) & LSR_DR)) {
        if ((clock() - start) > (clock_t) (timeout_secs * CLOCKS_PER_SEC)) {
            return -1;
        }
    }
    *out = (unsigned char) inp(base + REG_DATA);
    return 0;
}

/* CRC-16/XMODEM, poly 0x1021, init 0x0000 */
static unsigned int crc16_xmodem(const unsigned char *data, int len)
{
    unsigned int crc = 0;
    int i, j;

    for (i = 0; i < len; i++) {
        crc ^= ((unsigned int) data[i]) << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (unsigned int) ((crc << 1) ^ 0x1021);
            } else {
                crc = (unsigned int) (crc << 1);
            }
            crc &= 0xFFFF;
        }
    }
    return crc;
}

/* Byte i of block b is b*128+i masked to 8 bits, so the receiver can verify
 * content without a reference file. blk is the unwrapped block counter. */
static void fill_block(unsigned int blk, unsigned char *buf)
{
    int i;

    for (i = 0; i < BLOCK_SIZE; i++) {
        buf[i] = (unsigned char) ((((unsigned long) blk) * BLOCK_SIZE + i) & 0xFF);
    }
}

static int do_xmodem_tx(void)
{
    unsigned char data[BLOCK_SIZE];
    unsigned int blk;
    unsigned char wire_blk;
    unsigned char resp;
    int attempt;
    int got_start;
    clock_t start;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base);

    got_start = 0;
    start = clock();
    while (!got_start) {
        if (uart_recv_byte(com1_base, &resp, 1) == 0 && resp == XM_C) {
            got_start = 1;
            break;
        }
        if ((clock() - start) > (clock_t) (XM_SYNC_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
            printf("XMODEM TX FAIL: TIMEOUT WAITING FOR INITIAL 'C'\n");
            return 1;
        }
    }

    for (blk = 1; blk <= FILE_BLOCKS; blk++) {
        int acked = 0;

        /* Wire block number wraps at 256. blk drives the loop and content. */
        wire_blk = (unsigned char) (blk & 0xFF);

        fill_block(blk, data);

        for (attempt = 0; attempt < MAX_RETRIES; attempt++) {
            unsigned int crc;
            int i;

            if (uart_send_byte(com1_base, XM_SOH) != 0 ||
                uart_send_byte(com1_base, wire_blk) != 0 ||
                uart_send_byte(com1_base, (unsigned char) (0xFF - wire_blk)) != 0) {
                printf("XMODEM TX FAIL: TIMEOUT SENDING HEADER FOR BLOCK %u\n", blk);
                return 1;
            }

            for (i = 0; i < BLOCK_SIZE; i++) {
                if (uart_send_byte(com1_base, data[i]) != 0) {
                    printf("XMODEM TX FAIL: TIMEOUT SENDING DATA FOR BLOCK %u\n", blk);
                    return 1;
                }
            }

            if ((blk == CORRUPT_BLOCK) && (attempt == 0)) {
                /* wrong CRC to test NAK and retry */
                uart_send_byte(com1_base, 0x00);
                uart_send_byte(com1_base, 0x00);
            } else {
                crc = crc16_xmodem(data, BLOCK_SIZE);
                uart_send_byte(com1_base, (unsigned char) (crc >> 8));
                uart_send_byte(com1_base, (unsigned char) (crc & 0xFF));
            }

            if (uart_recv_byte(com1_base, &resp, XM_BYTE_TIMEOUT_SECS) != 0) {
                printf("XMODEM TX FAIL: TIMEOUT WAITING FOR ACK/NAK ON BLOCK %u\n", blk);
                return 1;
            }
            if (resp == XM_ACK) {
                acked = 1;
                break;
            }
            if (resp == XM_CAN) {
                printf("XMODEM TX FAIL: RECEIVER CANCELLED\n");
                return 1;
            }
            /* else NAK and retransmit */
        }

        if (!acked) {
            printf("XMODEM TX FAIL: BLOCK %u NEVER ACKED AFTER %d RETRIES\n", blk, MAX_RETRIES);
            return 1;
        }
    }

    {
        int eot_ok = 0;

        for (attempt = 0; attempt < MAX_RETRIES; attempt++) {
            uart_send_byte(com1_base, XM_EOT);
            if (uart_recv_byte(com1_base, &resp, XM_BYTE_TIMEOUT_SECS) == 0 && resp == XM_ACK) {
                eot_ok = 1;
                break;
            }
        }
        if (!eot_ok) {
            printf("XMODEM TX FAIL: EOT NEVER ACKED\n");
            return 1;
        }
    }

    printf("XMODEM TX DONE blocks=%d\n", FILE_BLOCKS);
    return 0;
}

static int do_xmodem_rx(void)
{
    static unsigned char got_data[FILE_BLOCKS][BLOCK_SIZE];
    unsigned int expected_blk = 1;  /* full counter, masked to a byte when compared */
    int blocks_received = 0;
    int total_mismatches = 0;
    int nak_count_corrupt_block = 0;
    unsigned char first_byte = 0;
    int have_pending;
    clock_t start;
    int i;

    printf("COM1 BASE=%04X\n", com1_base);
    uart_configure(com1_base);

    have_pending = 0;
    start = clock();
    while (!have_pending) {
        unsigned char b;

        uart_send_byte(com1_base, XM_C);
        if (uart_recv_byte(com1_base, &b, 2) == 0) {
            if ((b == XM_SOH) || (b == XM_EOT)) {
                first_byte = b;
                have_pending = 1;
                break;
            }
        }
        if ((clock() - start) > (clock_t) (XM_SYNC_TIMEOUT_SECS * CLOCKS_PER_SEC)) {
            printf("XMODEM RX FAIL: TIMEOUT WAITING FOR FIRST BLOCK\n");
            return 1;
        }
    }

    for (;;) {
        unsigned char soh_or_eot;

        if (have_pending) {
            soh_or_eot = first_byte;
            have_pending = 0;
        } else if (uart_recv_byte(com1_base, &soh_or_eot, XM_BYTE_TIMEOUT_SECS) != 0) {
            printf("XMODEM RX FAIL: TIMEOUT WAITING FOR NEXT BLOCK/EOT (expected blk %u)\n",
                   expected_blk);
            return 1;
        }

        if (soh_or_eot == XM_EOT) {
            uart_send_byte(com1_base, XM_ACK);
            break;
        }
        if (soh_or_eot == XM_CAN) {
            printf("XMODEM RX FAIL: SENDER CANCELLED\n");
            return 1;
        }
        if (soh_or_eot != XM_SOH) {
            uart_send_byte(com1_base, XM_NAK);
            continue;
        }

        {
            unsigned char blk, blk_comp;
            unsigned char buf[BLOCK_SIZE];
            unsigned char crc_hi, crc_lo;
            unsigned int recv_crc, calc_crc;
            int ok = 1;

            if (uart_recv_byte(com1_base, &blk, XM_BYTE_TIMEOUT_SECS) != 0) {
                ok = 0;
            }
            if (ok && (uart_recv_byte(com1_base, &blk_comp, XM_BYTE_TIMEOUT_SECS) != 0)) {
                ok = 0;
            }
            if (ok) {
                for (i = 0; i < BLOCK_SIZE; i++) {
                    if (uart_recv_byte(com1_base, &buf[i], XM_BYTE_TIMEOUT_SECS) != 0) {
                        ok = 0;
                        break;
                    }
                }
            }
            if (ok && (uart_recv_byte(com1_base, &crc_hi, XM_BYTE_TIMEOUT_SECS) != 0)) {
                ok = 0;
            }
            if (ok && (uart_recv_byte(com1_base, &crc_lo, XM_BYTE_TIMEOUT_SECS) != 0)) {
                ok = 0;
            }

            if (!ok) {
                printf("XMODEM RX FAIL: TIMEOUT MID-BLOCK (expected blk %u)\n", expected_blk);
                return 1;
            }

            if ((unsigned char) (blk + blk_comp) != 0xFF) {
                uart_send_byte(com1_base, XM_NAK);
                continue;
            }

            recv_crc = ((unsigned int) crc_hi << 8) | crc_lo;
            calc_crc = crc16_xmodem(buf, BLOCK_SIZE);

            if (recv_crc != calc_crc) {
                if (blk == CORRUPT_BLOCK) {
                    nak_count_corrupt_block++;
                }
                uart_send_byte(com1_base, XM_NAK);
                continue;
            }

            if (blk == (unsigned char) (expected_blk & 0xFF)) {
                if (blocks_received < FILE_BLOCKS) {
                    memcpy(got_data[blocks_received], buf, BLOCK_SIZE);
                }
                blocks_received++;
                expected_blk++;
                uart_send_byte(com1_base, XM_ACK);
            } else if (blk == (unsigned char) ((expected_blk - 1) & 0xFF)) {
                /* duplicate */
                uart_send_byte(com1_base, XM_ACK);
            } else {
                uart_send_byte(com1_base, XM_NAK);
            }
        }
    }

    if (blocks_received != FILE_BLOCKS) {
        printf("XMODEM FAIL: EXPECTED %d BLOCKS, GOT %d\n", FILE_BLOCKS, blocks_received);
        return 1;
    }

    for (i = 0; i < FILE_BLOCKS; i++) {
        unsigned char expect[BLOCK_SIZE];
        int j;

        fill_block((unsigned int) (i + 1), expect);
        for (j = 0; j < BLOCK_SIZE; j++) {
            if (got_data[i][j] != expect[j]) {
                total_mismatches++;
            }
        }
    }

    if (total_mismatches != 0) {
        printf("XMODEM FAIL: %d BYTE MISMATCHES AFTER TRANSFER\n", total_mismatches);
        return 1;
    }
    if (nak_count_corrupt_block == 0) {
        printf("XMODEM FAIL: CORRUPT-BLOCK RETRY NEVER EXERCISED (nak_count=0) -- test harness bug, not a real transfer failure\n");
        return 1;
    }

    printf("XMODEM OK blocks=%d bytes=%lu retries_seen=%d\n",
           blocks_received, ((unsigned long) blocks_received) * BLOCK_SIZE, nak_count_corrupt_block);
    return 0;
}

int main(int argc, char *argv[])
{
    com1_base = get_com1_base();

    if (argc != 2) {
        printf("USAGE: XMTEST <tx|rx>\n");
        return 2;
    }

    if (strcmp(argv[1], "tx") == 0) {
        return do_xmodem_tx();
    }
    if (strcmp(argv[1], "rx") == 0) {
        return do_xmodem_rx();
    }

    printf("USAGE: XMTEST <tx|rx>\n");
    return 2;
}
