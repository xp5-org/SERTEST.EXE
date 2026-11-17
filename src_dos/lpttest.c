/*
 * LPTTEST.EXE -- parallel port checks for DOS (Open Watcom, small model:
 * wcl -0 -ms -w4 lpttest.c). The checks are lpttest_core.c, shared with the Linux
 * puppet's lpt personality; this file is the console program around them.
 *
 *   LPTTEST [1|2|3 | base=378]
 *
 * One line per check: "LPT CHK <NAME> PASS|FAIL", then "LPT NOTE ..." and
 * "LPT DONE RESULT=PASS" or "LPT DONE RESULT=FAIL <n>". Exit code 0 on pass.
 * An on-line printer must be behind the port (86Box: lpt1_device = text_prt).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "sertest_defs.h"

static int lpt_res[16];

#include "lpttest_core.c"

static void lpt_set(int check, int state)
{
    static const char *const nm[] = { "PEND", "RUN", "PASS", "FAIL", "SKIP", "NC", "NA" };
    lpt_res[check] = state;
    if (state >= ST_PASS)
        printf("LPT CHK %-7s %s  %s\n", lpt_chk_name[check], nm[state], lpt_chk_text[check]);
}

int main(int argc, char **argv)
{
    int i, fails = 0, n = 1;
    for (i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "base=", 5))
            lpt_base = (unsigned int) strtoul(argv[i] + 5, NULL, 16);
        else if (argv[i][0] >= '1' && argv[i][0] <= '3')
            n = argv[i][0] - '0';
        else {
            printf("usage: LPTTEST [1|2|3 | base=378]\n");
            return 2;
        }
    }
    if (!lpt_base) {
        hw_wptr bda = (hw_wptr) hw_bda(0x0008);
        lpt_base = bda[n - 1] ? bda[n - 1] : 0x378;
    }
    hw_timer_begin();
    clk_init();
    printf("LPT HW base=%03X\n", lpt_base);
    lpt_run();
    for (i = 0; i < nnotes; i++)
        printf("LPT NOTE %s\n", notes[i]);
    for (i = 0; i < NLCHK; i++)
        if (lpt_res[i] == ST_FAIL)
            fails++;
    if (fails)
        printf("LPT DONE RESULT=FAIL %d\n", fails);
    else
        printf("LPT DONE RESULT=PASS\n");
    hw_timer_end();
    return fails ? 1 : 0;
}
