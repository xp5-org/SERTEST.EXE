/*
 * sertest_hw.h -- the machine interface of SERTEST.
 *
 * Everything the UART checks need from the machine goes through the names in
 * this header, so the checks themselves are plain C89 with no platform calls:
 *
 *   hw_in(p) / hw_out(p, v)     8-bit I/O port read / write
 *   hw_irq_off() / hw_irq_on()  CPU interrupt flag
 *   hw_bda_w(off)               16-bit word of the PC BIOS data area (0040:off),
 *                               0 where there is no BIOS data area
 *   HW_ISR(name)                declare an interrupt handler
 *   HW_ISR_PTR(n)               declare a variable n holding such a handler
 *   hw_timer_read()             free-running 16-bit DOWN counter, HW_TICK_HZ (1193182) ticks
 *                               per second, wraps every 65536 ticks (all timing is in these ticks)
 *   hw_timer_begin() / hw_timer_end()   set the counter up / put it back for the system
 *   hw_wptr / hw_lptr, hw_bda(off)   16/32-bit pointers into the PC BIOS data area
 *   hw_isr_get(vec) / hw_isr_set(vec, fn)   read / replace an interrupt vector
 *
 * One section per target below; a new target (Linux puppet, Atari, 68k Mac)
 * adds its own section and nothing else in the checks changes.
 */
#ifndef SERTEST_HW_H
#define SERTEST_HW_H

#if defined(__WATCOMC__) && defined(M_I86)
/* ---- MS-DOS, Open Watcom (wcl -0 -ms) ---- */
#include <conio.h>
#include <dos.h>
#include <i86.h>

#define hw_in(p)          inp(p)
#define hw_out(p, v)      outp((p), (v))
#define hw_irq_off()      _disable()
#define hw_irq_on()       _enable()
#define hw_bda_w(off)     (*(unsigned int far *) MK_FP(0x0040, (off)))
#define hw_bda_l(off)     (*(unsigned long far *) MK_FP(0x0040, (off)))
#define HW_ISR(name)      void __interrupt __far name(void)
#define HW_ISR_PTR(n)     void (__interrupt __far *n)(void)
#define hw_isr_get(vec)   _dos_getvect(vec)
#define hw_isr_set(vec, f) _dos_setvect((vec), (f))

#define HW_SCREEN_VAR(v)  static unsigned char far *v;
#define HW_IRQ_ARM(vec, bit, mask) do { \
        old_isr = hw_isr_get(vec); \
        hw_isr_set(vec, com_isr); \
        mask = (unsigned char) hw_in(PIC1_DATA); \
        hw_out(PIC1_DATA, mask & ~bit); \
    } while (0)
#define HW_IRQ_DISARM(vec, bit, mask) do { \
        hw_out(PIC1_DATA, (hw_in(PIC1_DATA) & ~bit) | (mask & bit)); \
        hw_isr_set(vec, old_isr); \
    } while (0)
#define HW_ISR_IIR()      hw_in(port + REG_IIR)
#define HW_IRQ_POLL(bit)  /* the interrupt handler runs by itself */

#define HW_TICK_HZ        1193182UL
#define hw_wptr           unsigned int far *
#define hw_lptr           unsigned long far *
#define hw_bda(off)       MK_FP(0x0040, (off))

#define HW_PIT_CH0        0x40
#define HW_PIT_CMD        0x43

#define hw_timer_begin()  hw_pit_mode(0x34)     /* mode 2: a clean 16-bit down counter */
#define hw_timer_end()    hw_pit_mode(0x36)     /* mode 3: what the BIOS expects */

#elif defined(__linux__) && defined(__i386__)
/* ---- Linux i386 user space, freestanding or hosted (the linuxpuppet personality) ----
 * Port I/O needs iopl(3) (the caller does that). User space cannot hook an interrupt
 * vector or clear the interrupt flag, so hw_irq_* are empty and the IRQ checks use
 * the interrupt controller's request register (see hw_irq_*_poll once added). There is
 * BIOS data area is whatever hw_bda_base points at: the port maps physical 0x400, or
 * leaves a zeroed table so COM base addresses fall back to the defaults. */
static inline unsigned char hw_in(unsigned short p)
{
    unsigned char v;
    __asm__ volatile("inb %1, %0" : "=a"(v) : "Nd"(p));
    return v;
}
static inline void hw_out(unsigned short p, unsigned char v)
{
    __asm__ volatile("outb %0, %1" : : "a"(v), "Nd"(p));
}
#define hw_irq_off()      ((void) 0)
#define hw_irq_on()       ((void) 0)
#define hw_wptr           volatile unsigned short *
#define hw_lptr           volatile unsigned int *
extern unsigned char *hw_bda_base;      /* the 0040:0000 area, or a zeroed table */
#define hw_bda(off)       ((void *) (hw_bda_base + (off)))
#define HW_SCREEN_VAR(v)
#define HW_TICK_HZ        1193182UL
#define HW_ISR(name)      void name(void)
#define HW_ISR_PTR(n)     void (*n)(void)
#define hw_isr_get(vec)   ((void (*)(void)) 0)
#define hw_isr_set(vec, f) ((void) (f))
/* The IRQ checks cannot take a real interrupt here, so the handler is called by hand when the
 * UART's own IIR reports a pending cause (bit 0 clear). That checks the UART's interrupt logic
 * (what IIR says, and that reading the cleared sources drops it) but not the line to the
 * interrupt controller: the controller's request register is edge-latched on real hardware and
 * cannot be cleared from user space, so a stale bit would make a later check pass wrongly.
 * hw_irr() is here for a port that wants to look at it anyway. */
static inline unsigned char hw_irr(void)
{
    hw_out(0x20, 0x0A);                 /* OCW3: next read of 0x20 is IRR */
    return hw_in(0x20);
}
#define HW_IRQ_ARM(vec, bit, mask) ((void) (vec), (void) (mask = (unsigned char) hw_in(PIC1_DATA)))
#define HW_IRQ_DISARM(vec, bit, mask) ((void) 0)
/* Reading IIR clears a THR-empty cause, so the value the poll read is handed to the handler. */
extern int hw_iir_pending;
unsigned char hw_iir_take(void);
#define HW_ISR_IIR()      (hw_iir_pending >= 0 ? hw_iir_take() : hw_in(port + REG_IIR))
#define HW_IRQ_POLL(bit)  do { int v_ = hw_in(port + REG_IIR); if (!(v_ & 1)) { hw_iir_pending = v_; com_isr(); } } while (0)
unsigned int hw_timer_read(void);
void hw_timer_begin(void);
void hw_timer_end(void);

#else
#error "sertest_hw.h: no machine interface for this target"
#endif

#endif

/* Function bodies. A program includes the header a second time with HW_IMPL
 * defined, at the point where it wants the code (the DOS build keeps it where
 * the timer code always was, so its object code stays the same). */
#ifdef HW_IMPL
#undef HW_IMPL
#if defined(__WATCOMC__) && defined(M_I86)
/* PIT channel 0 latched read. */
static unsigned int hw_timer_read(void)
{
    unsigned char lo, hi;
    hw_irq_off();
    hw_out(HW_PIT_CMD, 0x00);           /* latch channel 0 */
    lo = (unsigned char) hw_in(HW_PIT_CH0);
    hi = (unsigned char) hw_in(HW_PIT_CH0);
    hw_irq_on();
    return ((unsigned int) hi << 8) | lo;
}

static void hw_pit_mode(unsigned char mode_cmd)
{
    hw_irq_off();
    hw_out(HW_PIT_CMD, mode_cmd);       /* ch0, lobyte/hibyte */
    hw_out(HW_PIT_CH0, 0);
    hw_out(HW_PIT_CH0, 0);              /* reload 0 == 65536, the BIOS's own 18.2Hz */
    hw_irq_on();
}
#endif
#endif

/* detect_cpu(): where the CPU generation is told. Included from sertest_core.c at the
 * point detect_cpu used to be. */
#ifdef HW_IMPL_CPU
#undef HW_IMPL_CPU
#if defined(__WATCOMC__) && defined(M_I86)
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
#elif defined(__linux__) && defined(__i386__)
/* EFLAGS bit 18 (AC) is writable from a 486, bit 21 (ID) from a CPUID-capable part. */
static unsigned int hw_eflags_flip(unsigned int mask)
{
    unsigned int before, after;
    __asm__ volatile("pushfl; popl %0; movl %0, %1; xorl %2, %1; pushl %1; popfl; pushfl; popl %1; pushl %0; popfl"
                     : "=&r"(before), "=&r"(after) : "ir"(mask) : "cc");
    return (before ^ after) & mask;
}
static void detect_cpu(void)
{
    unsigned int a, b, c, d;
    if (!hw_eflags_flip(0x40000))
        cpu_name = "386";
    else if (!hw_eflags_flip(0x200000))
        cpu_name = "486";
    else {
        a = 1;
        __asm__ volatile("cpuid" : "+a"(a), "=b"(b), "=c"(c), "=d"(d));
        a = (a >> 8) & 0x0F;
        cpu_name = a == 4 ? "486" : a == 5 ? "P5" : "P6+";
    }
}
#endif
#endif
