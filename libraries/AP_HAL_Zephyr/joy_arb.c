/* see joy_arb.h */
#include "joy_arb.h"

#include <string.h>

bool joy_rfs_allowed(struct joy_rfs *r, uint32_t now_ms, bool radio_ok, bool joy_owns, uint32_t window_ms)
{
	if (radio_ok && !joy_owns) {
		r->seen = true;
		r->radio_cmd_ms = now_ms;
	}
	if (!r->seen) {
		return false;
	}
	if (!joy_owns) {
		return true;
	}
	return (uint32_t)(now_ms - r->radio_cmd_ms) <= window_ms;
}

void joy_arb_init(struct joy_arb *a)
{
	memset(a, 0, sizeof(*a));
	a->state = JOY_IDLE;
}

static bool joy_is_neutral(const joy_state_t *f)
{
	return f->throttle <= JOY_NEUTRAL_BAND && f->throttle >= -JOY_NEUTRAL_BAND &&
	       f->steering <= JOY_NEUTRAL_BAND && f->steering >= -JOY_NEUTRAL_BAND;
}

void joy_arb_step(struct joy_arb *a, const struct joy_arb_in *in, struct joy_arb_out *out)
{
	const uint32_t now = in->now_ms;
	const enum joy_arb_state before = a->state;

	memset(out, 0, sizeof(*out));

	/* frame freshness: CRC ok (decoded), < 100 ms, sequence step 1..5 from the previous frame */
	if (in->new_frame != NULL) {
		const uint8_t step = (uint8_t)(in->new_frame->seq - a->last_seq);

		a->seq_ok = !a->have_frame || (step >= 1u && step <= 5u);
		if (!a->seq_ok) {
			a->seq_errors++;
		}
		a->have_frame = true;
		a->frame = *in->new_frame;
		a->last_seq = in->new_frame->seq;
		a->last_frame_ms = now;
	}
	const bool fresh = a->have_frame && a->seq_ok && (uint32_t)(now - a->last_frame_ms) < JOY_FRESH_MS;
	const bool drive_ok = fresh && (a->frame.flags & JOY_F_GRIP) && (a->frame.flags & JOY_F_ARMED) &&
			      !(a->frame.flags & JOY_F_FAULT) && in->enable_pin;
	const bool disarmed_now = a->was_armed && !in->vehicle_armed;

	a->was_armed = in->vehicle_armed;

	/* neutral timer (on fresh frames only) */
	if (fresh && joy_is_neutral(&a->frame)) {
		if (!a->neutral) {
			a->neutral = true;
			a->neutral_since_ms = now;
		}
	} else {
		a->neutral = false;
	}

	switch (a->state) {
	case JOY_IDLE:
		if (drive_ok) {
			a->state = JOY_WAIT_NEUTRAL;
			a->remote_stop = false;
			/* the 300 ms of neutral count from the grip on, not from before it */
			a->neutral_since_ms = now;
		}
		break;
	case JOY_WAIT_NEUTRAL:
		if (in->remote_hold || disarmed_now) {
			a->state = JOY_STOPPED;
			a->remote_stop = true;
		} else if (!drive_ok) {
			a->state = JOY_STOPPED;
		} else if (a->neutral && in->vehicle_armed && (uint32_t)(now - a->neutral_since_ms) >= JOY_NEUTRAL_MS) {
			a->state = JOY_ACTIVE;
		}
		break;
	case JOY_ACTIVE:
		if (in->remote_hold || disarmed_now) {
			a->state = JOY_STOPPED;
			a->remote_stop = true;
		} else if (!drive_ok) {
			a->state = JOY_STOPPED;
			a->remote_stop = false;
		}
		break;
	case JOY_STOPPED:
		/* only a release of the hard-wired grip line ends a stop, once the vehicle holds */
		if (!in->enable_pin && (in->vehicle_in_hold || !in->vehicle_armed)) {
			a->state = JOY_IDLE;
			a->remote_stop = false;
		}
		break;
	}

	out->state = a->state;
	out->owns = (a->state != JOY_IDLE);
	out->driving = (a->state == JOY_ACTIVE);
	out->drive_ok = drive_ok;
	out->changed = (a->state != before);
	if (out->driving) {
		out->throttle = a->frame.throttle;
		out->steering = a->frame.steering;
		out->want_mode = JOY_WANT_MANUAL;
		out->status_bits |= JOY_S_ACTIVE;
	} else if (out->owns) {
		out->want_mode = JOY_WANT_HOLD;
		out->motor_neutral = true;
		out->status_bits |= JOY_S_HOLD;
		if (a->state == JOY_WAIT_NEUTRAL) {
			out->status_bits |= JOY_S_NEED_NEUTRAL;
		}
		if (a->remote_stop) {
			out->status_bits |= JOY_S_REMOTE_STOP;
		}
	}
}
