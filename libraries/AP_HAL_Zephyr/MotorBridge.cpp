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
#ifdef CONFIG_AP_HAL_ZEPHYR_FORCE_JOYSTICK
#include "Joystick.h"
#endif

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
    .welded_ms = CONFIG_AP_HAL_ZEPHYR_MB_WELDED_MS,
    .bus_min_mv = CONFIG_AP_HAL_ZEPHYR_MB_BUS_MIN_MV,
    .coast_ms = CONFIG_AP_HAL_ZEPHYR_MB_COAST_MS,
    .estop_debounce_ms = CONFIG_AP_HAL_ZEPHYR_MB_ESTOP_DEBOUNCE_MS,
    .i_filter_shift = CONFIG_AP_HAL_ZEPHYR_MB_I_FILTER_SHIFT,
    .duty_cap = CONFIG_AP_HAL_ZEPHYR_MB_DUTY_CAP,
    .max_gap_ms = CONFIG_AP_HAL_ZEPHYR_MB_MAX_GAP_MS,
};

constexpr uint32_t PERIOD_NS = 1000000000u / CONFIG_AP_HAL_ZEPHYR_MB_PWM_HZ;
/* ADC readings older than this are not used */
constexpr uint32_t SENSOR_MAX_AGE_MS = 20;

#if defined(CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS) && !defined(CONFIG_AP_HAL_ZEPHYR_MB_ALLOW_MOCK)
constexpr bool MOCK_BLOCK = true; /* a mock IMU must never drive the motors */
#else
constexpr bool MOCK_BLOCK = false;
#endif

struct mb_state st;
/* written by the vehicle (main thread), read by the 1 kHz process: aligned 32-bit stores */
volatile int32_t cmd[2];
volatile uint32_t last_cmd_ms[2];
volatile bool hard_latched;
volatile bool estop_now;
volatile bool arm_block = true;
bool hw_ok;
bool was_soft_armed;
uint32_t last_step_ms;
bool stepped;

/* ADC sampler thread -> 1 kHz process */
struct sample {
    uint32_t ms;
    uint32_t mv[N_ADC];
};
struct sample latest;
uint8_t fifo_pos[N_ADC]; /* where channel i lands in the sequence (channels are set up in ascending id) */
uint16_t adc_raw[N_ADC];
struct adc_sequence seq;

/* events from the 1 kHz process, reported by the sampler thread (not from the control step) */
volatile uint32_t ev_trip, ev_sense, ev_welded, ev_gap, ev_hw;

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

    /*
     * The TI ADC samples in the order the channels are set up (one sequencer step each), not in channel-id
     * order: set them up in ascending channel id and remember each one's position.
     */
    seq.buffer = adc_raw;
    seq.buffer_size = sizeof(adc_raw);
    seq.channels = 0;
    bool done[N_ADC] = {};
    for (uint8_t pos = 0; pos < N_ADC; pos++) {
        int best = -1;
        for (uint8_t i = 0; i < N_ADC; i++) {
            if (!done[i] && (best < 0 || adc[i].channel_id < adc[best].channel_id)) {
                best = i;
            }
        }
        done[best] = true;
        fifo_pos[best] = pos;
        ok &= adc_is_ready_dt(&adc[best]) && adc_channel_setup_dt(&adc[best]) == 0;
        seq.channels |= BIT(adc[best].channel_id);
        seq.resolution = adc[best].resolution;
    }
    return ok;
}

bool apply_safe()
{
    bool ok = gpio_pin_set_dt(&drv_arm, 0) == 0;
    for (uint8_t i = 0; i < 4; i++) {
        ok &= pwm_set_dt(&pwm[i], PERIOD_NS, 0) == 0;
    }
    for (uint8_t s = 0; s < 2; s++) {
        ok &= gpio_pin_set_dt(&en[s], 0) == 0;
        ok &= gpio_pin_set_dt(&brake[s], 0) == 0;
    }
    return ok;
}

/* false: a hardware write failed (the caller latches hard) */
bool apply(const struct mb_out &out)
{
    if (out.safe) {
        return apply_safe();
    }
    bool ok = true;
    for (uint8_t s = 0; s < 2; s++) {
        ok &= pwm_set_dt(&pwm[2 * s], PERIOD_NS, (uint32_t)((uint64_t)PERIOD_NS * out.pwm_fwd[s] / MB_DUTY_MAX)) == 0;
        ok &= pwm_set_dt(&pwm[2 * s + 1], PERIOD_NS, (uint32_t)((uint64_t)PERIOD_NS * out.pwm_rev[s] / MB_DUTY_MAX)) == 0;
        ok &= gpio_pin_set_dt(&en[s], out.enable[s]) == 0;
        ok &= gpio_pin_set_dt(&brake[s], out.brake_release[s]) == 0;
    }
    if (!ok) {
        return false;
    }
    return gpio_pin_set_dt(&drv_arm, out.drv_arm) == 0;
}

/* ADC sampler: a blocking read here never delays the control step, which only uses fresh samples */
void sampler_fn(void *, void *, void *)
{
    for (;;) {
        if (adc_read(adc[0].dev, &seq) == 0) {
            struct sample sm;
            sm.ms = k_uptime_get_32();
            for (uint8_t i = 0; i < N_ADC; i++) {
                int32_t mv = adc_raw[fifo_pos[i]];
                sm.mv[i] = (adc_raw_to_millivolts_dt(&adc[i], &mv) == 0 && mv > 0) ? (uint32_t)mv : 0u;
            }
            const unsigned key = irq_lock();
            latest = sm;
            irq_unlock(key);
        }
        if (ev_trip) {
            const uint32_t e = ev_trip;
            ev_trip = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motors stopped: over-current %s, disarm to reset",
                          (e & 1) ? ((e & 2) ? "L+R" : "L") : "R");
        }
        if (ev_sense) {
            const uint32_t e = ev_sense;
            ev_sense = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motors stopped: current sense %s implausible",
                          (e & 1) ? ((e & 2) ? "L+R" : "L") : "R");
        }
        if (ev_welded) {
            ev_welded = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Motor bus live with e-stop open: contactor?");
        }
        if (ev_gap) {
            ev_gap = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motors stopped: control step late, disarm to reset");
        }
        if (ev_hw) {
            ev_hw = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Motors stopped: output write failed");
        }
        k_sleep(K_MSEC(1));
    }
}

K_THREAD_STACK_DEFINE(sampler_stack, 2048);
struct k_thread sampler_thread;

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
    const uint32_t now = k_uptime_get_32();
    struct sample sm;

    if (!hw_ok) {
        return;
    }
    /* one step per millisecond: a repeated tick within the same ms is skipped */
    if (stepped && now == last_step_ms) {
        return;
    }
    stepped = true;
    last_step_ms = now;

    if (!soft_armed && was_soft_armed) {
        hard_latched = false; /* a hard latch, like every latched fault, clears on disarm */
    }
    was_soft_armed = soft_armed;

    {
        const unsigned key = irq_lock();
        sm = latest;
        irq_unlock(key);
    }

    in.now_ms = now;
    in.armed = soft_armed;
    estop_now = (gpio_pin_get_dt(&estop) != 0);
    in.estop_open = estop_now || hard_latched;
    in.last_cmd_ms[0] = last_cmd_ms[0];
    in.last_cmd_ms[1] = last_cmd_ms[1];
    in.cmd[0] = cmd[0];
    in.cmd[1] = cmd[1];
#ifdef CONFIG_AP_HAL_ZEPHYR_FORCE_JOYSTICK
    /* the joystick owns the chair without driving (grip released, stale, remote stop): 0 here, not later */
    if (Zephyr::Joystick::motor_neutral()) {
        in.cmd[0] = in.cmd[1] = 0;
    }
#endif
    in.sensors_ok = sm.ms != 0 && (uint32_t)(now - sm.ms) <= SENSOR_MAX_AGE_MS;
    if (in.sensors_ok) {
        for (uint8_t i = 0; i < 4; i++) {
            /* only the conducting high side of a half bridge reports: the side's current is the sum */
            in.current_ma[i / 2] += (uint32_t)(((uint64_t)sm.mv[i] * is_scale[i]) / 1000u);
        }
        in.vmot_mv = sm.mv[4] * vmot_scale;
        in.vbat_mv = sm.mv[5] * vmot_scale;
    }

    mb_step(&cfg, &st, &in, &out);
    if (!apply(out)) {
        hard_latched = true;
        (void)apply_safe();
        ev_hw = 1;
    }
    arm_block = out.arm_block || MOCK_BLOCK;

    for (uint8_t s = 0; s < 2; s++) {
        if (out.trip_event[s]) {
            ev_trip |= 1u << s;
        }
        if (out.sense_fault_event[s]) {
            ev_sense |= 1u << s;
        }
    }
    if (out.welded_event) {
        ev_welded = 1;
    }
    if (out.gap_event) {
        ev_gap = 1;
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
        printk("motor bridge: hardware not ready, outputs stay off, arming refused\n");
        return;
    }
    k_thread_create(&sampler_thread, sampler_stack, K_THREAD_STACK_SIZEOF(sampler_stack), sampler_fn, nullptr,
                    nullptr, nullptr, K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    k_thread_name_set(&sampler_thread, "ap_mb_adc");
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&process, &Process::tick, void));
    printk("motor bridge: up (servo channels %u/%u, %u Hz%s)\n", servo_ch[0], servo_ch[1],
           (unsigned)CONFIG_AP_HAL_ZEPHYR_MB_PWM_HZ, MOCK_BLOCK ? ", arming refused: mock sensors" : "");
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
    const uint8_t side = (ch == servo_ch[0]) ? 0 : 1;
    cmd[side] = c;
    last_cmd_ms[side] = k_uptime_get_32();
}

bool estop_active()
{
    return estop_now || st.estop_latched;
}

bool arming_blocked()
{
    return !hw_ok || arm_block;
}

} // namespace MotorBridge
} // namespace Zephyr

#else

extern "C" void ap_motor_bridge_latch_hard(void)
{
}

#endif
