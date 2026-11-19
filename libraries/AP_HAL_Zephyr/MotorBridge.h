/*
 * AP_HAL_Zephyr motor bridge glue (CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE): binds motor_bridge.c to the
 * ardupilot,motor-bridge devicetree node, the RCOutput motor channels and a 1 kHz timer process.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* hard stop until the vehicle disarms; ISR-safe (called from ap_hal_zephyr_outputs_safe() paths) */
void ap_motor_bridge_latch_hard(void);

#ifdef __cplusplus
}

namespace Zephyr {
namespace MotorBridge {
/* RCOutput: start the 1 kHz process (after the scheduler is up) */
void init();
/* RCOutput: is this channel a motor channel (then RCOutput does not drive it) */
bool is_motor_channel(uint8_t ch);
/* RCOutput: the vehicle wrote a pulse (us, 0 = off) to a motor channel */
void command(uint8_t ch, uint16_t pulse_us, bool enabled);
/* Util::safety_switch_state(): the e-stop is open or latched */
bool estop_active();
} // namespace MotorBridge
} // namespace Zephyr
#endif
