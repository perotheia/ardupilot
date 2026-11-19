#include "Util.h"

#include <zephyr/kernel.h>

#if defined(CONFIG_COMMON_LIBC_MALLOC)
// picolibc (Zephyr SDK >= 1.0 ships no newlib): malloc is Zephyr's common
// libc heap; mallinfo() would pull picolibc's own allocator in beside it
// (duplicate malloc/free at link).
#include <zephyr/sys/mem_stats.h>
extern "C" int malloc_runtime_stats_get(struct sys_memory_stats *stats);
#else
#include <malloc.h>
#endif

#include <AP_HAL_Zephyr/RCOutput.h>
#include <AP_HAL_Zephyr/Scheduler.h>
#ifdef CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE
#include "MotorBridge.h"
#endif

using namespace Zephyr;

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    const uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());
    _rtc_offset_us = time_utc_usec - now_us;
}

uint64_t Util::get_hw_rtc() const
{
    return k_ticks_to_us_floor64(k_uptime_ticks()) + _rtc_offset_us;
}

void Util::commandline_arguments(uint8_t &argc, char *const *&argv)
{
    auto *scheduler = static_cast<Zephyr::Scheduler *>(AP_HAL::get_HAL_mutable().scheduler);
    scheduler->commandline_arguments(argc, argv);
}

// base class returns 4096 which trips EKF3's memory check unconditionally;
// give it a real free-heap figure so it proceeds to actually allocate.
uint32_t Util::available_memory()
{
#if defined(CONFIG_COMMON_LIBC_MALLOC)
    struct sys_memory_stats stats;
    if (malloc_runtime_stats_get(&stats) == 0) {
        return (uint32_t)stats.free_bytes;
    }
#else
    struct mallinfo mi = mallinfo();
    if (mi.fordblks > 0) {
        return (uint32_t)mi.fordblks;
    }
#endif
    return CONFIG_HEAP_MEM_POOL_SIZE;
}

AP_HAL::Util::safety_state Util::safety_switch_state(void)
{
    auto *rcout = static_cast<Zephyr::RCOutput *>(AP_HAL::get_HAL().rcout);
    if (rcout == nullptr) {
        return SAFETY_NONE;
    }
#ifdef CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE
    /*
     * While DISARMED only, a motor-bridge reason not to arm (e-stop, latched fault, ...) reads as the safety
     * switch on: the arming checks refuse ("hardware safety switch"). Never while armed: Rover would drop
     * soft-armed and raise it again when the reason went away, which would look like a fresh arm to the
     * motor layer (an e-stop released while armed must not drive again before a real disarm + arm).
     */
    if (!get_soft_armed() && Zephyr::MotorBridge::arming_blocked()) {
        return SAFETY_DISARMED;
    }
#endif
    return rcout->safety_state();
}