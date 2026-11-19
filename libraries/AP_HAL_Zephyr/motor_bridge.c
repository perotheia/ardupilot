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
	sd->rest_since_ms = now;
	sd->zero_since_ms = now;
}

/* the fault detectors: over-current trip (hard) and current-sense plausibility (soft), both latched */
static void mb_side_faults(const struct mb_cfg *cfg, struct mb_side *sd, uint32_t i_ma, int s, struct mb_out *out)
{
	const bool enabled = (sd->brake != MB_BRAKE_APPLIED);
	bool implausible = false;

	if (cfg->i_trip_ma != 0 && i_ma > cfg->i_trip_ma) {
		if (++sd->over_trip_ms >= cfg->trip_ms && !sd->tripped) {
			sd->tripped = true;
			out->trip_event[s] = true;
		}
	} else {
		sd->over_trip_ms = 0;
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
	if (implausible) {
		if (++sd->implaus_ms >= cfg->is_fault_ms && !sd->sense_fault) {
			sd->sense_fault = true;
			out->sense_fault_event[s] = true;
		}
	} else {
		sd->implaus_ms = 0;
	}
}

/* a soft stop step: ramp to 0, brake on at 0 (or at the deadline), then safe; true = still ramping */
static bool mb_side_soft(const struct mb_cfg *cfg, struct mb_side *sd, uint32_t now)
{
	const int32_t step = cfg->soft_slew_per_ms ? cfg->soft_slew_per_ms : MB_DUTY_MAX;

	if (!sd->soft) {
		sd->soft = true;
		sd->soft_since_ms = now;
	}
	if (sd->duty == 0 || (uint32_t)(now - sd->soft_since_ms) >= cfg->soft_max_ms) {
		mb_side_safe(sd, now);
		return false;
	}
	sd->duty -= mb_clamp(sd->duty, -step, step);
	if (sd->duty == 0) {
		mb_side_safe(sd, now);
		return false;
	}
	return true;
}

void mb_step(const struct mb_cfg *cfg, struct mb_state *st, const struct mb_in *in, struct mb_out *out)
{
	const uint32_t now = in->now_ms;
	const bool fresh = (uint32_t)(now - in->last_cmd_ms) <= cfg->cmd_timeout_ms;
	bool any_active = false;

	memset(out, 0, sizeof(*out));

	/* e-stop: latched until disarm */
	if (in->estop_open) {
		st->estop_latched = true;
	}
	/* over-voltage on the higher of battery and motor bus, with hysteresis */
	if (cfg->vmot_max_mv == 0) {
		st->overvolt = false;
	} else {
		const uint32_t bus_mv = in->vmot_mv > in->vbat_mv ? in->vmot_mv : in->vbat_mv;

		if (bus_mv > cfg->vmot_max_mv) {
			st->overvolt = true;
		} else if (bus_mv + cfg->vmot_hyst_mv < cfg->vmot_max_mv) {
			st->overvolt = false;
		}
	}
	/* e-stop open while the bus stays live: welded contactor or regeneration */
	if (in->estop_open && cfg->vmot_live_mv != 0 && in->vmot_mv > cfg->vmot_live_mv) {
		out->welded_event = !st->welded_reported;
		st->welded_reported = true;
	} else {
		st->welded_reported = false;
	}

	/* latched faults clear when the vehicle disarms */
	if (!in->armed && st->was_armed) {
		st->estop_latched = false;
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
		int32_t target = mb_clamp(in->cmd[s], -MB_DUTY_MAX, MB_DUTY_MAX);

		mb_side_faults(cfg, sd, in->current_ma[s], s, out);

		/* HARD: immediate */
		if (in->estop_open || st->estop_latched || sd->tripped) {
			mb_side_safe(sd, now);
			continue;
		}

		/* SOFT: controlled ramp to 0 */
		if (!in->armed || !fresh || sd->sense_fault) {
			if (sd->duty != 0 || sd->soft) {
				if (mb_side_soft(cfg, sd, now)) {
					any_active = true;
					out->pwm_fwd[s] = (uint16_t)(sd->duty > 0 ? sd->duty : 0);
					out->pwm_rev[s] = (uint16_t)(sd->duty < 0 ? -sd->duty : 0);
					out->enable[s] = true;
					out->brake_release[s] = true;
				}
			} else {
				mb_side_safe(sd, now);
			}
			continue;
		}
		sd->soft = false;

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

		/* current foldback: scale the magnitude down in proportion to the excess */
		if (cfg->i_limit_ma != 0 && in->current_ma[s] > cfg->i_limit_ma && sd->duty != 0) {
			const int32_t cap = (int32_t)(((int64_t)mb_abs(sd->duty) * cfg->i_limit_ma) /
						       in->current_ma[s]);

			target = mb_clamp(target, -cap, cap);
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
		out->pwm_fwd[s] = (uint16_t)(sd->duty > 0 ? sd->duty : 0);
		out->pwm_rev[s] = (uint16_t)(sd->duty < 0 ? -sd->duty : 0);
		out->enable[s] = (sd->brake != MB_BRAKE_APPLIED);
		out->brake_release[s] = (sd->brake != MB_BRAKE_APPLIED);
	}

	/* the output stage stays armed (square wave) while a side runs or ramps down; never on an e-stop */
	if (any_active && !st->estop_latched) {
		st->drv_arm_level = !st->drv_arm_level;
	} else {
		st->drv_arm_level = false;
	}
	out->drv_arm = st->drv_arm_level;
	out->safe = !(any_active && !st->estop_latched);
	out->estop_latched = st->estop_latched;
	out->overvolt = st->overvolt;
}
