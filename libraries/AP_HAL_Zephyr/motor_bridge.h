/*
 * Driver-agnostic motor bridge state machine for AP_HAL_Zephyr (pure C, no Zephyr or ArduPilot types: the board
 * glue feeds it every millisecond and applies its outputs; host-tested in tests/).
 *
 * Per side: a signed command (-MB_DUTY_MAX..MB_DUTY_MAX) becomes a slew-limited duty that passes through zero
 * with a dead time on a reversal, is folded back above a current limit and latched off above a trip current, and
 * is gated by an electromagnetic brake (released before the duty rises, applied after the side has rested).
 *
 * Two stop classes (a person may be on the vehicle):
 *  HARD (immediate safe state, no ramp): e-stop open, over-current trip (that side, latched until disarm), and
 *    the HAL's own paths that bypass this code (shutdown, fault, stuck main loop: ap_hal_zephyr_outputs_safe()).
 *  SOFT (controlled ramp to 0 at soft_slew_per_ms, brakes applied once the duty is 0 or after soft_max_ms, then
 *    the safe state): no fresh command for cmd_timeout_ms (resumes when commands return), disarm while moving,
 *    current-sense plausibility fault (that side, latched until disarm). Vehicle-level failsafes (RC/GCS loss)
 *    command 0 through the vehicle (Rover HOLD), which ramps at the normal slew; a disarm then is a soft stop.
 * Safe state: drv_arm low, both PWM outputs 0, bridge enables low, brakes applied.
 *
 * DRV_ARM feeds a retriggerable monostable on the board (a rising edge at least every 5 ms keeps the output
 * stage enabled, ~10 ms timeout; a static level = all outputs off): out.drv_arm toggles on every step while the
 * bridge is meant to be live (500 Hz at a 1 kHz step), so a core, thread or timer that stops stepping cuts the
 * outputs in hardware within ~10 ms.
 * The e-stop (button loop, open = stop) latches until the vehicle disarms (a fresh arm is needed).
 * Over-voltage (BTS7960 lock-out 27.6..30 V), on the higher of the battery and the motor bus voltage: driving is
 * allowed only after a reading below vmot_max_mv - vmot_hyst_mv (27.0 V); above vmot_max_mv (27.4 V) the
 * command is held at 0 and the duty ramps down at the normal slew with the bridges kept enabled (no cut under
 * regeneration), resuming below 27.0 V. Motor bus above vmot_live_mv while the e-stop is open = welded
 * contactor / regeneration, reported.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_DUTY_MAX 10000 /* 100.00 % */
#define MB_SIDES    2

struct mb_cfg {
	uint16_t slew_per_ms;      /* duty units per ms (e.g. 20 = 0 -> 100 % in 0.5 s) */
	uint16_t reversal_dead_ms; /* duty held at 0 between directions */
	uint16_t brake_release_ms; /* time the brake needs to open before the duty may rise */
	uint16_t brake_hold_ms;    /* duty 0 and no command this long -> brake applied */
	uint16_t cmd_timeout_ms;   /* no fresh command this long -> safe state */
	uint32_t i_limit_ma;       /* above: duty foldback, 0 = no limit */
	uint32_t i_trip_ma;        /* above for trip_ms: side latched off until disarm, 0 = no trip */
	uint16_t trip_ms;
	uint16_t soft_slew_per_ms; /* soft stop ramp (e.g. 34 = 100 % -> 0 in ~0.3 s) */
	uint16_t soft_max_ms;      /* a soft stop ends in the safe state after this at the latest */
	/* current-sense plausibility, 0 = check off: */
	uint32_t is_idle_max_ma;   /* reading above this while the bridge is disabled = fault */
	uint16_t is_dead_duty;     /* |duty| above this ... */
	uint32_t is_dead_min_ma;   /* ... with a reading below this = fault (open sense / wiring) */
	uint16_t is_fault_ms;      /* for this long */
	uint32_t vmot_max_mv;      /* no driving above (BTS7960 locks out at 27.6 V), 0 = no check */
	uint32_t vmot_hyst_mv;
	uint32_t vmot_live_mv;     /* bus considered live above, for the e-stop cross-check */
};

enum mb_brake { MB_BRAKE_APPLIED, MB_BRAKE_RELEASING, MB_BRAKE_RELEASED };

struct mb_side {
	int32_t duty;              /* signed, what is driven */
	enum mb_brake brake;
	uint32_t brake_since_ms;   /* start of MB_BRAKE_RELEASING */
	uint32_t rest_since_ms;    /* duty 0 and command 0 since */
	uint32_t zero_since_ms;    /* duty reached 0 at (reversal dead time) */
	uint32_t over_trip_ms;     /* consecutive ms above i_trip_ma */
	uint32_t implaus_ms;       /* consecutive ms of an implausible current reading */
	uint32_t soft_since_ms;    /* start of the current soft stop */
	bool soft;                 /* in a soft stop */
	bool tripped;              /* hard, latched until disarm */
	bool sense_fault;          /* soft, latched until disarm */
};

struct mb_state {
	struct mb_side side[MB_SIDES];
	bool was_armed;
	bool estop_latched;        /* until disarm */
	bool overvolt;             /* with hysteresis */
	bool drv_arm_level;        /* the square wave */
	bool welded_reported;
};

struct mb_in {
	uint32_t now_ms;
	bool armed;
	bool estop_open;
	uint32_t last_cmd_ms;      /* when cmd[] was last written by the vehicle */
	int32_t cmd[MB_SIDES];     /* -MB_DUTY_MAX..MB_DUTY_MAX */
	uint32_t current_ma[MB_SIDES];
	uint32_t vmot_mv;          /* motor bus voltage (after the contactor) */
	uint32_t vbat_mv;          /* battery voltage */
};

struct mb_out {
	uint16_t pwm_fwd[MB_SIDES]; /* 0..MB_DUTY_MAX, never both > 0 on one side */
	uint16_t pwm_rev[MB_SIDES];
	bool enable[MB_SIDES];
	bool brake_release[MB_SIDES];
	bool drv_arm;               /* level for this step: toggles while live, low when safe */
	bool safe;                  /* the whole bridge is in the safe state */
	bool estop_latched;
	bool overvolt;
	bool welded_event;          /* e-stop open but the motor bus still live (first step of it) */
	bool trip_event[MB_SIDES];  /* a side tripped in this step */
	bool sense_fault_event[MB_SIDES]; /* a side's current sense became implausible in this step */
};

void mb_init(struct mb_state *st);
void mb_step(const struct mb_cfg *cfg, struct mb_state *st, const struct mb_in *in, struct mb_out *out);

#ifdef __cplusplus
}
#endif
