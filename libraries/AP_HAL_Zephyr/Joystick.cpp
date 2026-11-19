/*
 * Force joystick glue (see Joystick.h, joy_arb.h, wc-hardware docs/JOYSTICK-FRAME.md v1).
 *
 * 1 kHz timer process: drain the UART (non-blocking FIFO reads), parse frames, read the grip line, step the
 * arbitration, publish steering and throttle (RCMAP, 1500 +- 400 us; neutral while the joystick owns but does
 * not drive) as the priority RC source on those two channels only (ELRS mode/aux channels and their overrides
 * stay live: a HOLD from the radio still arrives), send R5F_STATUS at 4 Hz.
 * Main thread: apply the requested mode (HOLD / MANUAL) through AP_Vehicle and report state changes. Mode changes
 * from anywhere else pass ap_board_mode_change_allowed(): while the joystick owns the chair only HOLD is
 * accepted (stop always wins) and an accepted HOLD is a remote stop; DISARM is never blocked.
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_AP_HAL_ZEPHYR_FORCE_JOYSTICK)

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>

#include "Joystick.h"
#include "RCInput.h"
/* joy_frame.h is a verbatim copy of the stick firmware's C header (no C++ guards) */
extern "C" {
#include "joy_frame.h"
}
#include "joy_arb.h"
#ifdef CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE
#include "MotorBridge.h"
#endif

extern const AP_HAL::HAL &hal;

#define JOY_NODE DT_INST(0, ardupilot_force_joystick)

namespace {

constexpr uint8_t MODE_MANUAL = 0; // Rover Mode::Number
constexpr uint8_t MODE_HOLD = 4;

const struct device *const uart = DEVICE_DT_GET(DT_PHANDLE(JOY_NODE, uart));
const struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(JOY_NODE, enable_gpios);

struct joy_arb arb;
joy_parser_t parser;
volatile bool neutral_req;
volatile bool owns;
volatile uint8_t want_mode = JOY_WANT_NONE;
bool remote_hold_ev;            /* main thread -> timer: an accepted HOLD from elsewhere (atomic exchange) */
bool estop_was;                 /* SRV_Channels emergency stop, last tick (timer thread) */
volatile bool own_request;      /* main thread: the mode change in flight is the joystick's own */
volatile uint8_t state_ev = 0xff;
uint32_t last_status_ms;
bool ready;
bool init_failed;

/* stick -1000..1000 -> us through the channel's own RCn_MIN/TRIM/MAX/REVERSED, so that Rover's pwm_to_angle()
   gives back the stick (dead zone aside) whatever the radio's calibration is */
uint16_t stick_to_pwm(uint8_t idx, int16_t v)
{
    const RC_Channel *c = rc().channel(idx);
    if (c == nullptr) {
        return 1500u;
    }
    const int32_t trim = c->get_radio_trim();
    int32_t s = c->get_reverse() ? -(int32_t)v : (int32_t)v;
    int32_t pwm = s >= 0 ? trim + (s * (c->get_radio_max() - trim)) / 1000 : trim + (s * (trim - c->get_radio_min())) / 1000;
    return (uint16_t)constrain_int32(pwm, 800, 2200);
}

class Process {
public:
    void tick();
};
Process process;

void Process::tick()
{
    struct joy_arb_in in = {};
    struct joy_arb_out out;
    joy_state_t frame;
    bool have = false;
    uint8_t b;

    in.now_ms = AP_HAL::millis();
    /* non-blocking: what the UART FIFO holds (115200 baud = ~12 bytes per ms) */
    for (int i = 0; i < 64 && uart_fifo_read(uart, &b, 1) == 1; i++) {
        if (joy_parser_push(&parser, b) == JOY_TYPE_STATE &&
            joy_decode_state(joy_parser_payload(&parser), joy_parser_len(&parser), &frame) == 0) {
            have = true; /* the newest complete frame of this tick */
        }
    }
    in.new_frame = have ? &frame : nullptr;
    in.enable_pin = gpio_pin_get_dt(&enable) > 0;
    in.vehicle_armed = hal.util->get_soft_armed();
    AP_Vehicle *v = AP_Vehicle::get_singleton();
    in.vehicle_in_hold = v != nullptr && v->get_mode() == MODE_HOLD;
    in.remote_hold = __atomic_exchange_n(&remote_hold_ev, false, __ATOMIC_SEQ_CST);
    /* the software e-stop (RC aux MOTOR_ESTOP, a GCS) is a remote stop too: on its rising edge */
    const bool estop = SRV_Channels::get_emergency_stop();
    if (estop && !estop_was) {
        in.remote_hold = true;
    }
    estop_was = estop;

    joy_arb_step(&arb, &in, &out);

    owns = out.owns;
    neutral_req = out.motor_neutral;
    want_mode = out.want_mode;
    if (out.changed) {
        state_ev = out.state;
    }
    /* steering and throttle (RCMAP), through their channels' calibration; all other channels stay the radio's */
    const RCMapper *map = AP::rcmap();
    if (map != nullptr) {
        const uint8_t cs = (uint8_t)(map->roll() - 1), ct = (uint8_t)(map->throttle() - 1);
        static_cast<Zephyr::RCInput *>(hal.rcin)->set_priority_input(
            cs, stick_to_pwm(cs, out.steering), stick_to_pwm(cs, 0),
            ct, stick_to_pwm(ct, out.throttle), stick_to_pwm(ct, 0), out.owns);
    }

    /* R5F_STATUS at 4 Hz (10 bytes fit the UART FIFO: no blocking) */
    if ((uint32_t)(in.now_ms - last_status_ms) >= 250u) {
        joy_status_t st = {arb.last_seq, out.status_bits};
#ifdef CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE
        if (Zephyr::MotorBridge::estop_active()) {
            st.state |= JOY_S_ESTOP_OPEN;
        }
#endif
        uint8_t f[JOY_STATUS_FRAME_BYTES];
        const size_t n = joy_encode_status(&st, f);
        (void)uart_fifo_fill(uart, f, (int)n);
        last_status_ms = in.now_ms;
    }
}

const char *state_text(uint8_t s)
{
    switch (s) {
    case JOY_WAIT_NEUTRAL: return "grip: HOLD, return stick to neutral";
    case JOY_ACTIVE: return "active (MANUAL)";
    case JOY_STOPPED: return "stopped: HOLD, release grip, then neutral + re-grip";
    default: return "released";
    }
}

} // namespace

/* Rover::set_mode() veto: while the joystick owns the chair, only HOLD (and its own requests) pass */
extern "C" bool ap_board_mode_change_allowed(uint8_t new_mode, uint8_t reason)
{
    (void)reason;
    if (own_request || !owns) {
        return true;
    }
    if (new_mode == MODE_HOLD) {
        /* stop always wins: accepted, and the joystick latches a remote stop at its next tick; until then
           nothing may ask for MANUAL again and the motor layer holds neutral */
        __atomic_store_n(&remote_hold_ev, true, __ATOMIC_SEQ_CST);
        want_mode = JOY_WANT_HOLD;
        neutral_req = true;
        return true;
    }
    return false;
}

namespace Zephyr {
namespace Joystick {

void init()
{
    joy_arb_init(&arb);
    joy_parser_init(&parser);
    if (!device_is_ready(uart) || !gpio_is_ready_dt(&enable) ||
        gpio_pin_configure_dt(&enable, GPIO_INPUT) != 0) {
        printk("joystick: UART or enable line not ready: joystick off\n");
        init_failed = true;
        return;
    }
    ready = true;
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&process, &Process::tick, void));
    printk("joystick: up\n");
}

bool motor_neutral()
{
    return neutral_req;
}

void main_service()
{
    static uint32_t last_warn_ms;
    if (!ready) {
        if (init_failed && AP_HAL::millis() - last_warn_ms > 30000u) {
            last_warn_ms = AP_HAL::millis();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Joystick: UART or grip line not ready, joystick OFF");
        }
        return;
    }
    AP_Vehicle *v = AP_Vehicle::get_singleton();
    if (v == nullptr) {
        return;
    }
    const uint8_t want = want_mode;
    const uint8_t mode = v->get_mode();
    uint8_t target = 0xff;

    if (want == JOY_WANT_HOLD && mode != MODE_HOLD) {
        target = MODE_HOLD;
    } else if (want == JOY_WANT_MANUAL && mode != MODE_MANUAL && !__atomic_load_n(&remote_hold_ev, __ATOMIC_SEQ_CST)) {
        target = MODE_MANUAL;
    }
    if (target != 0xff) {
        own_request = true;
        (void)v->set_mode(target, ModeReason::RC_COMMAND);
        own_request = false;
    }
    if (state_ev != 0xff) {
        const uint8_t s = state_ev;
        state_ev = 0xff;
        GCS_SEND_TEXT(s == JOY_STOPPED ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO, "Joystick %s", state_text(s));
    }
}

} // namespace Joystick
} // namespace Zephyr

#endif
