/* see motor_bridge.h */
#include "motor_bridge.h"

#include <string.h>

static int32_t mb_clamp(int32_t v, int32_t lo, int32_t hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static int32_t mb_sign(int32_t v)
{
	return (v > 0) - (v < 0);
}

static int32_t mb_abs(int32_t v)
{
	return v < 0 ? -v : v;
}

void mb_init(struct mb_state *st)
{
	memset(st, 0, sizeof(*st));
	for (int s = 0; s < MB_SIDES; s++) {
		st->side[s].brake = MB_BRAKE_APPLIED;
	}
	st->overvolt = true; /* until a bus reading below the enable threshold */
}

static void mb_side_safe(struct mb_side *sd, uint32_t now)
{
	sd->duty = 0;
	sd->brake = MB_BRAKE_APPLIED;
	sd->soft = false;
	sd->coasting = false;
	sd->rest_since_ms = now;
	sd->zero_since_ms = now;
}

/* leaky counter: up while the condition holds, down otherwise */
static void mb_leak(uint32_t *count, bool cond)
{
	if (cond) {
		(*count)++;
	} else if (*count > 0) {
		(*count)--;
	}
}

/* over-current trip (hard) and current-sense plausibility (soft), both latched while armed */
static void mb_side_faults(const struct mb_cfg *cfg, struct mb_side *sd, uint32_t i_ma, int s, struct mb_out *out)
{
	const bool enabled = (sd->brake != MB_BRAKE_APPLIED);
	bool implausible = false;

	mb_leak(&sd->over_trip_ms, cfg->i_trip_ma != 0 && i_ma > cfg->i_trip_ma);
	if (cfg->i_trip_ma != 0 && sd->over_trip_ms >= cfg->trip_ms && !sd->tripped) {
		sd->tripped = true;
		out->trip_event[s] = true;
	}

	if (cfg->is_fault_ms != 0) {
		if (!enabled && cfg->is_idle_max_ma != 0 && i_ma > cfg->is_idle_max_ma) {
			implausible = true; /* current with the bridge off */
		}
		if (enabled && cfg->is_dead_duty != 0 && mb_abs(sd->duty) > cfg->is_dead_duty &&
		    i_ma < cfg->is_dead_min_ma) {
			implausible = true; /* no current at high duty */
		}
	}
	mb_leak(&sd->implaus_ms, implausible);
	if (cfg->is_fault_ms != 0 && sd->implaus_ms >= cfg->is_fault_ms && !sd->sense_fault) {
		sd->sense_fault = true;
		out->sense_fault_event[s] = true;
	}
}

/*
 * A soft stop step: ramp to 0, then coast with the bridge enabled at duty 0 (short brake) for coast_ms, then
 * the safe state (parking brake on); soft_max_ms bounds the whole stop. true = still active.
 */
static bool mb_side_soft(const struct mb_cfg *cfg, struct mb_side *sd, uint32_t now)
{
	const int32_t step = cfg->soft_slew_per_ms ? cfg->soft_slew_per_ms : MB_DUTY_MAX;

	if (!sd->soft) {
		sd->soft = true;
		sd->soft_since_ms = now;
		sd->coasting = (sd->duty == 0);
		sd->coast_since_ms = now;
	}
	if ((uint32_t)(now - sd->soft_since_ms) >= cfg->soft_max_ms || sd->brake == MB_BRAKE_APPLIED ||
	    (sd->brake == MB_BRAKE_RELEASING && sd->duty == 0)) {
		mb_side_safe(sd, now);
		return false;
	}
	if (!sd->coasting) {
		sd->duty -= mb_clamp(sd->duty, -step, step);
		if (sd->duty == 0) {
			sd->coasting = true;
			sd->coast_since_ms = now;
		}
		return true;
	}
	if ((uint32_t)(now - sd->coast_since_ms) >= cfg->coast_ms) {
		mb_side_safe(sd, now);
		return false;
	}
	return true;
}

static void mb_out_side(const struct mb_side *sd, int s, struct mb_out *out)
{
	out->pwm_fwd[s] = (uint16_t)(sd->duty > 0 ? sd->duty : 0);
	out->pwm_rev[s] = (uint16_t)(sd->duty < 0 ? -sd->duty : 0);
	out->enable[s] = (sd->brake != MB_BRAKE_APPLIED);
	out->brake_release[s] = (sd->brake != MB_BRAKE_APPLIED);
}

void mb_step(const struct mb_cfg *cfg, struct mb_state *st, const struct mb_in *in, struct mb_out *out)
{
	const uint32_t now = in->now_ms;
	bool fresh = true;
	bool any_active = false;
	bool any_trip = false, any_sense = false;
	const uint32_t bus_mv = in->vmot_mv > in->vbat_mv ? in->vmot_mv : in->vbat_mv;
	const bool bus_bad = !in->sensors_ok || (cfg->bus_min_mv != 0 && in->vbat_mv < cfg->bus_min_mv);

	memset(out, 0, sizeof(*out));

	/* a late step: the dead-man stage may have dropped the outputs, the duty state is no longer real */
	if (st->stepped && cfg->max_gap_ms != 0 && (uint32_t)(now - st->last_step_ms) > cfg->max_gap_ms) {
		out->gap_event = !st->gap_latched;
		st->gap_latched = true;
	}
	st->stepped = true;
	st->last_step_ms = now;

	for (int s = 0; s < MB_SIDES; s++) {
		fresh = fresh && (uint32_t)(now - in->last_cmd_ms[s]) <= cfg->cmd_timeout_ms;
	}

	/* e-stop: debounced (a short glitch must not slam the brakes), then latched until disarm */
	if (in->estop_open) {
		if (st->estop_count < cfg->estop_debounce_ms) {
			st->estop_count++;
		}
		if (st->estop_count >= (cfg->estop_debounce_ms ? cfg->estop_debounce_ms : 1) && !st->estop_latched) {
			st->estop_latched = true;
			st->estop_since_ms = now;
		}
	} else {
		st->estop_count = 0;
	}

	/* over-voltage on the higher of battery and motor bus, with hysteresis; no valid reading = no driving */
	if (cfg->vmot_max_mv == 0) {
		st->overvolt = false;
	} else if (in->sensors_ok) {
		if (bus_mv > cfg->vmot_max_mv) {
			st->overvolt = true;
		} else if (bus_mv + cfg->vmot_hyst_mv < cfg->vmot_max_mv) {
			st->overvolt = false;
		}
	}

	/* e-stop open while the motor bus stays live for welded_ms: welded contactor or regeneration */
	if (in->estop_open && st->estop_latched && cfg->vmot_live_mv != 0 && in->sensors_ok &&
	    in->vmot_mv > cfg->vmot_live_mv && (uint32_t)(now - st->estop_since_ms) >= cfg->welded_ms) {
		out->welded_event = !st->welded_reported;
		st->welded_reported = true;
	} else if (!in->estop_open) {
		st->welded_reported = false;
	}

	/* latched faults: clear on disarm; while disarmed they also clear once their condition is gone */
	if (!in->armed && st->was_armed) {
		st->estop_latched = in->estop_open;
		st->gap_latched = false;
		for (int s = 0; s < MB_SIDES; s++) {
			st->side[s].tripped = false;
			st->side[s].sense_fault = false;
			st->side[s].over_trip_ms = 0;
			st->side[s].implaus_ms = 0;
		}
	}
	st->was_armed = in->armed;

	for (int s = 0; s < MB_SIDES; s++) {
		struct mb_side *sd = &st->side[s];

		/* foldback input: filtered current */
		sd->i_filt_ma += (in->current_ma[s] >> cfg->i_filter_shift) - (sd->i_filt_ma >> cfg->i_filter_shift);
		mb_side_faults(cfg, sd, in->sensors_ok ? in->current_ma[s] : 0, s, out);
		if (!in->armed) {
			if (sd->over_trip_ms == 0) {
				sd->tripped = false;
			}
			if (sd->implaus_ms == 0) {
				sd->sense_fault = false;
			}
			if (!in->estop_open) {
				st->estop_latched = false;
			}
			st->gap_latched = false; /* nothing is driven while disarmed */
		}
		any_trip = any_trip || sd->tripped;
		any_sense = any_sense || sd->sense_fault;
	}

	/* a watchdog stop must not end in a jump to the old command: wait for a neutral command */
	if (!fresh) {
		st->need_neutral = true;
	} else if (in->cmd[0] == 0 && in->cmd[1] == 0) {
		st->need_neutral = false;
	}

	for (int s = 0; s < MB_SIDES; s++) {
		struct mb_side *sd = &st->side[s];
		const int32_t cap = cfg->duty_cap ? cfg->duty_cap : MB_DUTY_MAX;
		int32_t target = mb_clamp(in->cmd[s], -cap, cap);

		/* HARD: immediate, on the faulty side; the other side of a trip stops softly */
		if (st->estop_latched || st->gap_latched || sd->tripped) {
			mb_side_safe(sd, now);
			continue;
		}

		/* SOFT: controlled ramp to 0, coast, brake */
		if (!in->armed || !fresh || st->need_neutral || any_trip || any_sense || bus_bad) {
			if (sd->duty != 0 || sd->soft) {
				if (mb_side_soft(cfg, sd, now)) {
					any_active = true;
					mb_out_side(sd, s, out);
				}
			} else {
				mb_side_safe(sd, now);
			}
			continue;
		}
		sd->soft = false;
		sd->coasting = false;

		/* over-voltage: no acceleration, ramp down at the normal slew, bridges stay enabled */
		if (st->overvolt) {
			target = 0;
		}

		/* brake: open it (bridge enabled, duty held at 0) before driving */
		if (target != 0 && !st->overvolt && sd->brake == MB_BRAKE_APPLIED) {
			sd->brake = MB_BRAKE_RELEASING;
			sd->brake_since_ms = now;
		}
		if (sd->brake == MB_BRAKE_RELEASING &&
		    (uint32_t)(now - sd->brake_since_ms) >= cfg->brake_release_ms) {
			sd->brake = MB_BRAKE_RELEASED;
		}
		if (sd->brake != MB_BRAKE_RELEASED) {
			target = 0;
		}

		/* current foldback on the filtered current, in proportion to the excess */
		if (cfg->i_limit_ma != 0 && sd->i_filt_ma > cfg->i_limit_ma && sd->duty != 0) {
			const int32_t lim = (int32_t)(((int64_t)mb_abs(sd->duty) * cfg->i_limit_ma) / sd->i_filt_ma);

			target = mb_clamp(target, -lim, lim);
		}

		/* reversal: down to 0, hold the dead time, then the other way */
		if (sd->duty != 0 && target != 0 && mb_sign(target) != mb_sign(sd->duty)) {
			target = 0;
		} else if (sd->duty == 0 && target != 0 &&
			   (uint32_t)(now - sd->zero_since_ms) < cfg->reversal_dead_ms) {
			target = 0;
		}

		/* slew */
		{
			const int32_t step = cfg->slew_per_ms ? cfg->slew_per_ms : MB_DUTY_MAX;
			const int32_t prev = sd->duty;

			sd->duty += mb_clamp(target - sd->duty, -step, step);
			if (prev != 0 && sd->duty == 0) {
				sd->zero_since_ms = now;
			}
		}

		/* rest: duty 0 with no command (or over-voltage) for the hold time -> brake back on */
		if (sd->duty == 0 && (in->cmd[s] == 0 || st->overvolt)) {
			if ((uint32_t)(now - sd->rest_since_ms) >= cfg->brake_hold_ms) {
				sd->brake = MB_BRAKE_APPLIED;
			}
		} else {
			sd->rest_since_ms = now;
		}

		any_active = true;
		mb_out_side(sd, s, out);
	}

	/* the output stage stays armed (square wave) while a side runs or stops softly; never on an e-stop */
	if (any_active && !st->estop_latched && !st->gap_latched) {
		st->drv_arm_level = !st->drv_arm_level;
	} else {
		st->drv_arm_level = false;
	}
	out->drv_arm = st->drv_arm_level;
	out->safe = !(any_active && !st->estop_latched && !st->gap_latched);
	out->estop_latched = st->estop_latched;
	out->overvolt = st->overvolt;
	out->arm_block = st->estop_latched || in->estop_open || st->gap_latched || any_trip || any_sense ||
			 st->overvolt || bus_bad;
}
