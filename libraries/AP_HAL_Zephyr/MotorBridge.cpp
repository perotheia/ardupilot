/*
 * Motor bridge glue: devicetree wiring + RCOutput commands + ADC + 1 kHz timer process around the pure state
 * machine in motor_bridge.c (see that header for the behaviour and the stop classes).
 *
 * Output order: going safe, DRV_ARM is dropped FIRST (the dead-man stage then disables every output in
 * hardware), then PWM 0, bridge enables low, brakes applied; going live, PWM / enables / brakes are written
 * first and DRV_ARM's square wave last. The hard latch from ap_hal_zephyr_outputs_safe() (shutdown, fault,
 * stuck main loop) holds the safe state until the vehicle disarms; the board hook has also turned the GPIO2
 * pins into inputs, which no later write undoes (re-arming needs a reboot).
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_AP_HAL_ZEPHYR_MOTOR_BRIDGE)

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#include <GCS_MAVLink/GCS.h>

#include "MotorBridge.h"
#include "motor_bridge.h"

extern const AP_HAL::HAL &hal;

#define MB_NODE DT_INST(0, ardupilot_motor_bridge)

BUILD_ASSERT(DT_PROP_LEN(MB_NODE, pwms) == 4, "motor bridge: 4 pwms (L fwd, L rev, R fwd, R rev)");
BUILD_ASSERT(DT_PROP_LEN(MB_NODE, io_channels) == 6, "motor bridge: 6 io-channels (4 current, motor bus, battery)");
BUILD_ASSERT(DT_PROP_LEN(MB_NODE, is_ma_per_mv) == 4, "motor bridge: 4 current scales");

namespace {

const struct pwm_dt_spec pwm[4] = {
    PWM_DT_SPEC_GET_BY_IDX(MB_NODE, 0), PWM_DT_SPEC_GET_BY_IDX(MB_NODE, 1),
    PWM_DT_SPEC_GET_BY_IDX(MB_NODE, 2), PWM_DT_SPEC_GET_BY_IDX(MB_NODE, 3),
};
const struct gpio_dt_spec en[2] = {
    GPIO_DT_SPEC_GET_BY_IDX(MB_NODE, enable_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(MB_NODE, enable_gpios, 1),
};
const struct gpio_dt_spec brake[2] = {
    GPIO_DT_SPEC_GET_BY_IDX(MB_NODE, brake_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(MB_NODE, brake_gpios, 1),
};
const struct gpio_dt_spec drv_arm = GPIO_DT_SPEC_GET(MB_NODE, drv_arm_gpios);
const struct gpio_dt_spec estop = GPIO_DT_SPEC_GET(MB_NODE, estop_gpios);
constexpr uint8_t N_ADC = 6;
const struct adc_dt_spec adc[N_ADC] = {
    ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 0), ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 1), ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 2),
    ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 3), ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 4), ADC_DT_SPEC_GET_BY_IDX(MB_NODE, 5),
};
const uint32_t is_scale[4] = DT_PROP(MB_NODE, is_ma_per_mv);
const uint32_t vmot_scale = DT_PROP(MB_NODE, vmot_mv_per_mv);
const uint8_t servo_ch[2] = {DT_PROP_BY_IDX(MB_NODE, servo_channels, 0),
                             DT_PROP_BY_IDX(MB_NODE, servo_channels, 1)};

const struct mb_cfg cfg = {
    .slew_per_ms = CONFIG_AP_HAL_ZEPHYR_MB_SLEW_PER_MS,
    .reversal_dead_ms = CONFIG_AP_HAL_ZEPHYR_MB_REVERSAL_DEAD_MS,
    .brake_release_ms = CONFIG_AP_HAL_ZEPHYR_MB_BRAKE_RELEASE_MS,
    .brake_hold_ms = CONFIG_AP_HAL_ZEPHYR_MB_BRAKE_HOLD_MS,
    .cmd_timeout_ms = CONFIG_AP_HAL_ZEPHYR_MB_CMD_TIMEOUT_MS,
    .i_limit_ma = CONFIG_AP_HAL_ZEPHYR_MB_I_LIMIT_MA,
    .i_trip_ma = CONFIG_AP_HAL_ZEPHYR_MB_I_TRIP_MA,
    .trip_ms = CONFIG_AP_HAL_ZEPHYR_MB_TRIP_MS,
    .soft_slew_per_ms = CONFIG_AP_HAL_ZEPHYR_MB_SOFT_SLEW_PER_MS,
    .soft_max_ms = CONFIG_AP_HAL_ZEPHYR_MB_SOFT_MAX_MS,
    .is_idle_max_ma = CONFIG_AP_HAL_ZEPHYR_MB_IS_IDLE_MAX_MA,
    .is_dead_duty = CONFIG_AP_HAL_ZEPHYR_MB_IS_DEAD_DUTY,
    .is_dead_min_ma = CONFIG_AP_HAL_ZEPHYR_MB_IS_DEAD_MIN_MA,
    .is_fault_ms = CONFIG_AP_HAL_ZEPHYR_MB_IS_FAULT_MS,
    .vmot_max_mv = CONFIG_AP_HAL_ZEPHYR_MB_VMOT_MAX_MV,
    .vmot_hyst_mv = CONFIG_AP_HAL_ZEPHYR_MB_VMOT_HYST_MV,
    .vmot_live_mv = CONFIG_AP_HAL_ZEPHYR_MB_VMOT_LIVE_MV,
};

constexpr uint32_t PERIOD_NS = 1000000000u / CONFIG_AP_HAL_ZEPHYR_MB_PWM_HZ;

struct mb_state st;
/* written by the vehicle (main thread), read by the 1 kHz process */
volatile int32_t cmd[2];
volatile uint32_t last_cmd_ms;
volatile bool hard_latched;
volatile bool estop_now;
bool hw_ok;
bool was_soft_armed;
uint16_t adc_raw[N_ADC];
struct adc_sequence seq;

bool setup_hw()
{
    bool ok = true;

    /* everything off before anything is enabled; DRV_ARM first */
    ok &= gpio_is_ready_dt(&drv_arm) && gpio_pin_configure_dt(&drv_arm, GPIO_OUTPUT_INACTIVE) == 0;
    for (uint8_t i = 0; i < 4; i++) {
        ok &= pwm_is_ready_dt(&pwm[i]) && pwm_set_dt(&pwm[i], PERIOD_NS, 0) == 0;
    }
    for (uint8_t s = 0; s < 2; s++) {
        ok &= gpio_is_ready_dt(&en[s]) && gpio_pin_configure_dt(&en[s], GPIO_OUTPUT_INACTIVE) == 0;
        ok &= gpio_is_ready_dt(&brake[s]) && gpio_pin_configure_dt(&brake[s], GPIO_OUTPUT_INACTIVE) == 0;
    }
    ok &= gpio_is_ready_dt(&estop) && gpio_pin_configure_dt(&estop, GPIO_INPUT) == 0;

    seq.buffer = adc_raw;
    seq.buffer_size = sizeof(adc_raw);
    seq.channels = 0;
    for (uint8_t i = 0; i < N_ADC; i++) {
        ok &= adc_is_ready_dt(&adc[i]) && adc_channel_setup_dt(&adc[i]) == 0;
        seq.channels |= BIT(adc[i].channel_id);
        seq.resolution = adc[i].resolution;
    }
    return ok;
}

void apply_safe()
{
    gpio_pin_set_dt(&drv_arm, 0);
    for (uint8_t i = 0; i < 4; i++) {
        pwm_set_dt(&pwm[i], PERIOD_NS, 0);
    }
    for (uint8_t s = 0; s < 2; s++) {
        gpio_pin_set_dt(&en[s], 0);
        gpio_pin_set_dt(&brake[s], 0);
    }
}

void apply(const struct mb_out &out)
{
    if (out.safe) {
        apply_safe();
        return;
    }
    for (uint8_t s = 0; s < 2; s++) {
        pwm_set_dt(&pwm[2 * s], PERIOD_NS, (uint32_t)((uint64_t)PERIOD_NS * out.pwm_fwd[s] / MB_DUTY_MAX));
        pwm_set_dt(&pwm[2 * s + 1], PERIOD_NS, (uint32_t)((uint64_t)PERIOD_NS * out.pwm_rev[s] / MB_DUTY_MAX));
        gpio_pin_set_dt(&en[s], out.enable[s]);
        gpio_pin_set_dt(&brake[s], out.brake_release[s]);
    }
    gpio_pin_set_dt(&drv_arm, out.drv_arm);
}

/* the ADC channels are converted in channel-id order into adc_raw */
uint32_t adc_mv(uint8_t idx)
{
    uint8_t pos = 0;
    for (uint8_t i = 0; i < N_ADC; i++) {
        if (adc[i].channel_id < adc[idx].channel_id) {
            pos++;
        }
    }
    int32_t mv = adc_raw[pos];
    if (adc_raw_to_millivolts_dt(&adc[idx], &mv) != 0 || mv < 0) {
        return 0;
    }
    return (uint32_t)mv;
}

class Process {
public:
    void tick();
};
Process process;

void Process::tick()
{
    struct mb_in in = {};
    struct mb_out out;
    const bool soft_armed = hal.util->get_soft_armed();

    if (!hw_ok) {
        return;
    }
    if (!soft_armed && was_soft_armed) {
        hard_latched = false; /* a hard latch, like every latched fault, clears on disarm */
    }
    was_soft_armed = soft_armed;

    in.now_ms = k_uptime_get_32();
    in.armed = soft_armed;
    estop_now = (gpio_pin_get_dt(&estop) != 0);
    in.estop_open = estop_now || hard_latched;
    in.last_cmd_ms = last_cmd_ms;
    in.cmd[0] = cmd[0];
    in.cmd[1] = cmd[1];
    if (adc_read(adc[0].dev, &seq) == 0) {
        for (uint8_t i = 0; i < 4; i++) {
            /* the active half bridge of each side is the one that reports: sum both */
            in.current_ma[i / 2] += (uint32_t)(((uint64_t)adc_mv(i) * is_scale[i]) / 1000u);
        }
        in.vmot_mv = adc_mv(4) * vmot_scale;
        in.vbat_mv = adc_mv(5) * vmot_scale;
    }
    /*
     * An ADC failure leaves 0 mA (the plausibility check turns that into a soft stop at high duty) and 0 V: the
     * over-voltage state is kept from the last good reading.
     */

    mb_step(&cfg, &st, &in, &out);
    apply(out);

    for (uint8_t s = 0; s < 2; s++) {
        if (out.trip_event[s]) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motor %c: over-current trip, disarm to reset", s ? 'R' : 'L');
        }
        if (out.sense_fault_event[s]) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motor %c: current sense implausible, stopped", s ? 'R' : 'L');
        }
    }
    if (out.welded_event) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Motor bus live with e-stop open: contactor?");
    }
}

} // namespace

void ap_motor_bridge_latch_hard(void)
{
    hard_latched = true;
}

namespace Zephyr {
namespace MotorBridge {

void init()
{
    mb_init(&st);
    hw_ok = setup_hw();
    if (!hw_ok) {
        printk("motor bridge: hardware not ready, outputs stay off\n");
        return;
    }
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&process, &Process::tick, void));
    printk("motor bridge: up (servo channels %u/%u, %u Hz)\n", servo_ch[0], servo_ch[1],
           (unsigned)CONFIG_AP_HAL_ZEPHYR_MB_PWM_HZ);
}

bool is_motor_channel(uint8_t ch)
{
    return ch == servo_ch[0] || ch == servo_ch[1];
}

void command(uint8_t ch, uint16_t pulse_us, bool enabled)
{
    int32_t c = 0;

    /* 1000..2000 us, 1500 = stop, +-10 us dead band */
    if (enabled && pulse_us != 0) {
        const int32_t d = (int32_t)pulse_us - 1500;
        if (d > 10 || d < -10) {
            c = d * (MB_DUTY_MAX / 500);
        }
        c = c > MB_DUTY_MAX ? MB_DUTY_MAX : (c < -MB_DUTY_MAX ? -MB_DUTY_MAX : c);
    }
    cmd[ch == servo_ch[0] ? 0 : 1] = c;
    last_cmd_ms = k_uptime_get_32();
}

bool estop_active()
{
    return estop_now || st.estop_latched;
}

} // namespace MotorBridge
} // namespace Zephyr

#else

extern "C" void ap_motor_bridge_latch_hard(void)
{
}

#endif
