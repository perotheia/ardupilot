#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <cstdarg>
#include <cstdio>

#include <AP_HAL/AP_HAL.h>
#include "HAL_Zephyr_Class.h"
#include "CycleClock.h"
#include "SafeState.h"

#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>

// LOG_PANIC() (flush deferred log messages) without log_ctrl.h, which does
// not compile as C++; nothing to flush in minimal mode or without logging
#if defined(CONFIG_LOG) && !defined(CONFIG_LOG_MODE_MINIMAL) && !defined(CONFIG_USERSPACE)
extern "C" void z_impl_log_panic(void);
#define AP_LOG_PANIC() z_impl_log_panic()
#else
#define AP_LOG_PANIC() ((void)0)
#endif
// kernel-internal header (kernel/include): declared here
extern "C" FUNC_NORETURN void arch_system_halt(unsigned int reason);

// The default safe-state hook: nothing (see SafeState.h for why it does not
// call hal.rcout->force_safety_on()). Weak: a board overrides it.
extern "C" __attribute__((weak)) void ap_hal_zephyr_outputs_safe(void)
{
}

/*
 * Zephyr fatal errors (CPU exceptions, kernel panics, k_oops, a failed stack
 * sentinel): outputs safe first, then what the kernel's weak default does.
 * Defined in this file because it is always linked (AP_HAL::get_HAL()): the
 * linker would not pull a separate archive member only to replace the
 * kernel's weak symbol.
 */
extern "C" void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
    ARG_UNUSED(esf);
    ap_hal_zephyr_outputs_safe();
    printk("AP_HAL_Zephyr: fatal error %u, outputs safe\n", reason);
    AP_LOG_PANIC();
#if defined(CONFIG_RESET_ON_FATAL_ERROR)
    printk("Resetting system\n");
    sys_reboot(SYS_REBOOT_WARM);
#else
    printk("Halting system\n");
    arch_system_halt(reason);
#endif
    CODE_UNREACHABLE;
}

#ifdef CONFIG_AP_HAL_ZEPHYR_CYCLE_CLOCK
/*
 * Microsecond clock from the hardware cycle counter. One instance, guarded
 * by irq_lock(): callable from every thread and from ISRs on a single core
 * (the build refuses SMP below).
 *
 * ASSUMPTION: this is called at least once per wrap period of the 32-bit
 * counter (2^32 / sys_clock_hw_cycles_per_sec(): 223.7 s at the AI-64's
 * 19.2 MHz). Scheduler::_run_timer_procs() reads the clock on every pass of
 * the 1 kHz timer thread, whatever else is registered; a system where that
 * thread is dead for minutes has lost its time base anyway.
 */
#ifdef CONFIG_SMP
#error "AP_HAL_ZEPHYR_CYCLE_CLOCK: the irq_lock() guard is single-core only"
#endif
static Zephyr::CycleClock s_cycle_clock;

static uint64_t cycle_clock_us(void)
{
    const unsigned int key = irq_lock();
    const uint32_t counter = k_cycle_get_32();
    if (!s_cycle_clock.started()) {
        // epoch: the kernel uptime at the first call, as micros64() reported
        // before (time starts near 0 at boot)
        s_cycle_clock.start(sys_clock_hw_cycles_per_sec(), counter,
                            k_ticks_to_us_floor64(k_uptime_ticks()));
    }
    const uint64_t cycles = s_cycle_clock.update(counter);
    irq_unlock(key);
    return s_cycle_clock.to_us(cycles);
}
#endif

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
    char buffer[256];
    va_list ap;

    // before anything that can fault or block (the formatting, the console)
    ap_hal_zephyr_outputs_safe();

    va_start(ap, errormsg);
    vsnprintf(buffer, sizeof(buffer), errormsg, ap);
    va_end(ap);

    printk("AP_HAL panic: %s\n", buffer);

    for (;;) {
        k_sleep(K_FOREVER);
    }
}

uint32_t micros()
{
    return static_cast<uint32_t>(micros64());
}

uint32_t millis()
{
    return static_cast<uint32_t>(millis64());
}

#ifdef CONFIG_AP_HAL_ZEPHYR_CYCLE_CLOCK
uint64_t micros64()
{
    return cycle_clock_us();
}

uint64_t millis64()
{
    // the same clock as micros64(): the two never disagree
    return cycle_clock_us() / 1000U;
}
#else
// tick resolution (1 ms at CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000)
uint64_t micros64()
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

uint64_t millis64()
{
    return k_uptime_get();
}
#endif

} // namespace AP_HAL

static HAL_Zephyr hal_zephyr;

const AP_HAL::HAL &AP_HAL::get_HAL()
{
    return hal_zephyr;
}

AP_HAL::HAL &AP_HAL::get_HAL_mutable()
{
    return hal_zephyr;
}