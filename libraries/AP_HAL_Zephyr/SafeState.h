/*
 * AP_HAL_Zephyr safe-state hook.
 *
 * ap_hal_zephyr_outputs_safe() puts every actuator output of the board into
 * its safe state (motors unpowered) and leaves it there. The HAL calls it
 * first in every path that stops or may have lost the control loop:
 *
 *   - AP_HAL::panic()                                    (system.cpp)
 *   - k_sys_fatal_error_handler(), any Zephyr fatal error (system.cpp)
 *   - the main loop made no progress for
 *     CONFIG_AP_HAL_ZEPHYR_MAIN_STUCK_MS, from the timer thread
 *                                                        (Scheduler.cpp)
 *   - a board's remoteproc shutdown hook (e.g. boards/beagle/
 *     beaglebone_ai64/ai64_board.c), before the core is stopped
 *
 * CONTRACT for an implementation: callable from ISR and fault context and
 * with interrupts locked, more than once, at any time after reset (also
 * before the drivers are initialised). So: no locks, no sleeping, no driver
 * API that may take either; raw register writes or equally simple code.
 *
 * The HAL's default (system.cpp, weak) does nothing: RCOutput::
 * force_safety_on() goes through pwm_set() and gpio_pin_set_dt(), which are
 * not ISR-safe for every driver. A board with motors overrides it with a
 * strong definition.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void ap_hal_zephyr_outputs_safe(void);

#ifdef __cplusplus
}
#endif
