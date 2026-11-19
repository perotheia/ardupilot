#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <cstdarg>
#include <cstdio>

#include <AP_HAL/AP_HAL.h>
#include "HAL_Zephyr_Class.h"
#include "CycleClock.h"

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