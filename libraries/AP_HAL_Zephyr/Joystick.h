/*
 * AP_HAL_Zephyr force joystick source (CONFIG_AP_HAL_ZEPHYR_FORCE_JOYSTICK): frames from the stick's UART and the
 * hard-wired grip line through joy_arb.c (see there for the rules); the result reaches ArduRover as a priority
 * RC source (RCInput::set_priority_input) plus mode requests (HOLD / MANUAL) from the main thread.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace Zephyr {
namespace Joystick {
void init();
/* the motor layer forces neutral (the joystick owns the chair and does not drive) */
bool motor_neutral();
/* main thread, once per loop: mode requests, messages */
void main_service();
} // namespace Joystick
} // namespace Zephyr
