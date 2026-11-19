/*
 * Force joystick arbitration (pure C, host-tested): decides whether the grip-enabled joystick is the chair's
 * driving source, what it commands, and which vehicle mode it asks for. Frame spec v1 (wc-hardware
 * docs/JOYSTICK-FRAME.md): drive only if the frame is fresh (CRC ok, < 100 ms old, sequence step 1..5) AND GRIP
 * AND ARMED (the stick's own) AND !FAULT AND the hard-wired enable line is high.
 *
 * States:
 *  IDLE          not the driving source; modes and other sources untouched
 *  WAIT_NEUTRAL  grip taken: HOLD requested, output 0, MANUAL after 300 ms of neutral with the vehicle armed
 *  ACTIVE        MANUAL, throttle/steering from the stick
 *  STOPPED       drive conditions lost or a REMOTE STOP (HOLD/DISARM/e-stop from ELRS, a GCS or Linux): output
 *                0, HOLD requested; leaves only through a grip release (enable line low) once the vehicle is in
 *                HOLD or disarmed (until then the joystick keeps owning: neutral, no overrides), then a re-grip
 *                at neutral (no automatic resume)
 * While not IDLE the joystick owns the chair: RC overrides are suppressed and mode changes from elsewhere are
 * refused except HOLD (stop always wins); DISARM is never blocked.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "joy_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

enum joy_arb_state { JOY_IDLE, JOY_WAIT_NEUTRAL, JOY_ACTIVE, JOY_STOPPED };
enum joy_want_mode { JOY_WANT_NONE, JOY_WANT_HOLD, JOY_WANT_MANUAL };

#define JOY_FRESH_MS      100
#define JOY_NEUTRAL_MS    300
#define JOY_NEUTRAL_BAND  50 /* |throttle|, |steering| <= 50 of 1000 = neutral */

/* R5F_STATUS state bits: JOY_S_* in joy_frame.h (spec v1, b4 NEED_NEUTRAL and b5 REMOTE_STOP since e785750) */

struct joy_arb {
	enum joy_arb_state state;
	bool have_frame;
	bool seq_ok;
	uint8_t last_seq;
	uint32_t last_frame_ms;
	joy_state_t frame;
	uint32_t neutral_since_ms;
	bool neutral;
	bool remote_stop;   /* the current STOPPED came from elsewhere */
	bool was_armed;
	uint32_t seq_errors;
};

struct joy_arb_in {
	uint32_t now_ms;
	const joy_state_t *new_frame; /* a frame decoded (CRC ok) in this step, or NULL */
	bool enable_pin;              /* P8_26 */
	bool vehicle_armed;
	bool vehicle_in_hold;         /* the vehicle's mode is HOLD (confirmed) */
	bool remote_hold;             /* a HOLD request from elsewhere was accepted in this step */
};

struct joy_arb_out {
	enum joy_arb_state state;
	bool owns;                    /* not IDLE: suppress overrides, filter mode changes */
	bool driving;                 /* ACTIVE */
	int16_t throttle, steering;   /* -1000..1000, 0 unless driving */
	enum joy_want_mode want_mode;
	bool motor_neutral;           /* the motor layer forces 0 (owns and not driving) */
	uint8_t status_bits;          /* JOY_S_ACTIVE, JOY_S_HOLD, JOY_S_NEED_NEUTRAL, JOY_S_REMOTE_STOP */
	bool drive_ok;                /* the frame/pin conditions of the spec */
	bool changed;                 /* state changed in this step (for messages) */
};

void joy_arb_init(struct joy_arb *a);
void joy_arb_step(struct joy_arb *a, const struct joy_arb_in *in, struct joy_arb_out *out);

#ifdef __cplusplus
}
#endif
