/* host tests for joy_arb.c + joy_frame.c (the files the firmware compiles), run by run.sh */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../joy_frame.c"
#include "../joy_arb.c"

static struct joy_arb arb;
static struct joy_arb_in in;
static struct joy_arb_out out;
static joy_state_t fr;
static uint32_t now = 1000;
static int checks;

static void invariants(void)
{
	if (!out.driving) {
		assert(out.throttle == 0 && out.steering == 0);
	}
	if (out.driving) {
		assert(out.drive_ok && out.want_mode == JOY_WANT_MANUAL && !out.motor_neutral);
	}
	if (out.owns && !out.driving) {
		assert(out.want_mode == JOY_WANT_HOLD && out.motor_neutral);
	}
	if (!out.owns) {
		assert(out.want_mode == JOY_WANT_NONE && !out.motor_neutral);
	}
	checks++;
}

/* one ms; a frame every 10 ms (100 Hz) unless stopped */
static void step(bool send)
{
	in.now_ms = now;
	in.new_frame = NULL;
	if (send && now % 10 == 0) {
		fr.seq++;
		in.new_frame = &fr;
	}
	joy_arb_step(&arb, &in, &out);
	invariants();
	in.remote_hold = false;
	/* the vehicle follows the requested mode one step later */
	if (out.want_mode == JOY_WANT_HOLD) {
		in.vehicle_in_hold = true;
	} else if (out.want_mode == JOY_WANT_MANUAL) {
		in.vehicle_in_hold = false;
	}
	now++;
}

static void steps(int n, bool send = true)
{
	for (int i = 0; i < n; i++) {
		step(send);
	}
}

static void reset(void)
{
	joy_arb_init(&arb);
	memset(&in, 0, sizeof(in));
	memset(&fr, 0, sizeof(fr));
	fr.flags = JOY_F_GRIP | JOY_F_ARMED | JOY_F_CALIBRATED;
	in.enable_pin = true;
	in.vehicle_armed = true;
}

static void grip_to_active(void)
{
	fr.throttle = fr.steering = 0;
	steps(20);
	assert(out.state == JOY_WAIT_NEUTRAL && (out.status_bits & JOY_S_NEED_NEUTRAL));
	steps(300);
	assert(out.state == JOY_ACTIVE);
}

static void test_codec(void)
{
	assert(joy_crc16((const uint8_t *)"123456789", 9) == 0x29B1);
	joy_state_t s = {}, d = {};
	s.seq = 7; s.flags = JOY_F_GRIP | JOY_F_ARMED; s.throttle = -321; s.steering = 999; s.fx_cN = -1234;
	uint8_t buf[JOY_STATE_FRAME_BYTES + 8];
	const uint8_t junk[] = {0x00, 0xA5, 0x13, 0xA5, 0x5A, 0x01, 0xFF};
	size_t n = joy_encode_state(&s, buf);
	joy_parser_t p;
	joy_parser_init(&p);
	int got = 0;
	for (size_t i = 0; i < sizeof(junk); i++) (void)joy_parser_push(&p, junk[i]); /* garbage, a false start */
	for (size_t i = 0; i < n; i++) {
		if (joy_parser_push(&p, buf[i]) > 0) {
			assert(joy_parser_len(&p) == JOY_STATE_LEN);
			assert(joy_decode_state(joy_parser_payload(&p), joy_parser_len(&p), &d) == 0);
			got++;
		}
	}
	assert(got == 1 && d.seq == 7 && d.throttle == -321 && d.steering == 999 && d.fx_cN == -1234);
	buf[6] ^= 0x10; /* corrupted: no frame */
	got = 0;
	for (size_t i = 0; i < n; i++) got += joy_parser_push(&p, buf[i]) > 0;
	assert(got == 0 && p.crc_errors >= 1);
	puts("codec: CRC check value, encode/decode, resync after garbage, CRC reject: ok");
}

static void test_neutral_rule(void)
{
	reset();
	fr.throttle = 800; /* gripped with the stick deflected */
	steps(1000);
	assert(out.state == JOY_WAIT_NEUTRAL && out.throttle == 0 && out.want_mode == JOY_WANT_HOLD);
	fr.throttle = 0;
	steps(299);
	assert(out.state == JOY_WAIT_NEUTRAL);
	steps(15);
	assert(out.state == JOY_ACTIVE && out.want_mode == JOY_WANT_MANUAL);
	fr.throttle = 600; fr.steering = -200;
	steps(20);
	assert(out.throttle == 600 && out.steering == -200);
	puts("grip -> HOLD, MANUAL only after 300 ms of neutral: ok");
}

static void test_grip_release_no_resume(void)
{
	reset();
	grip_to_active();
	fr.throttle = 700;
	steps(20);
	in.enable_pin = false; /* hard line released (frame still says grip) */
	steps(1);
	assert(out.state == JOY_STOPPED && out.throttle == 0 && out.motor_neutral && out.owns);
	steps(1); /* the vehicle has switched to HOLD: ownership ends */
	assert(out.state == JOY_IDLE);
	/* a vehicle that does not reach HOLD keeps the joystick owning (neutral, no overrides) */
	in.enable_pin = true;
	fr.throttle = 0;
	steps(330);
	assert(out.state == JOY_ACTIVE);
	in.enable_pin = false;
	steps(1);
	in.vehicle_in_hold = false; /* stuck in MANUAL */
	arb.state = JOY_STOPPED;
	in.now_ms = now;
	in.new_frame = NULL;
	joy_arb_step(&arb, &in, &out);
	assert(out.state == JOY_STOPPED && out.motor_neutral);
	puts("grip release: output 0 at once, ownership only ends with the vehicle in HOLD: ok");
}

static void test_stale_and_fault(void)
{
	reset();
	grip_to_active();
	fr.throttle = 500;
	steps(20);
	steps(120, false); /* no frames */
	assert(out.state == JOY_STOPPED && out.throttle == 0 && out.motor_neutral);
	steps(500); /* frames back, grip still held: no resume */
	assert(out.state == JOY_STOPPED);
	in.enable_pin = false;
	steps(5);
	assert(out.state == JOY_IDLE);
	in.enable_pin = true;
	fr.throttle = 0;
	steps(330);
	assert(out.state == JOY_ACTIVE);
	fr.flags |= JOY_F_FAULT;
	steps(11);
	assert(out.state == JOY_STOPPED);
	puts("stale frames / stick fault: STOPPED, resume only via release + neutral + re-grip: ok");
}

static void test_sequence(void)
{
	reset();
	grip_to_active();
	fr.seq += 9; /* a jump of 10: not fresh until the next in-sequence frame */
	steps(10);
	steps(1);
	assert(out.state == JOY_STOPPED);
	puts("sequence jump > 5: not fresh -> STOPPED: ok");
}

/* pascal's rule: STOP ALWAYS WINS */
static void test_remote_stop_wins(void)
{
	reset();
	grip_to_active();
	fr.throttle = 1000; /* full deflection */
	steps(20);
	assert(out.throttle == 1000);
	in.remote_hold = true; /* HOLD from ELRS / a GCS */
	steps(1);
	assert(out.state == JOY_STOPPED && out.throttle == 0 && out.want_mode == JOY_WANT_HOLD &&
	       (out.status_bits & JOY_S_REMOTE_STOP));
	steps(1000); /* grip held, stick deflected and then neutral: stays stopped */
	fr.throttle = 0;
	steps(1000);
	assert(out.state == JOY_STOPPED && out.owns);
	in.enable_pin = false; /* release */
	steps(5);
	assert(out.state == JOY_IDLE);
	in.enable_pin = true; /* re-grip, neutral */
	steps(330);
	assert(out.state == JOY_ACTIVE);
	/* DISARM variant (also the Linux e-stop path: it disarms) */
	in.vehicle_armed = false;
	steps(1);
	assert(out.state == JOY_STOPPED && (out.status_bits & JOY_S_REMOTE_STOP));
	in.vehicle_armed = true; /* re-armed, grip still held */
	steps(1000);
	assert(out.state == JOY_STOPPED);
	in.enable_pin = false;
	steps(5);
	in.enable_pin = true;
	steps(330);
	assert(out.state == JOY_ACTIVE);
	/* re-grip with the stick already centred: still 300 ms of neutral AFTER the grip */
	in.enable_pin = false; steps(5); in.enable_pin = true; steps(250);
	assert(out.state == JOY_WAIT_NEUTRAL);
	steps(60);
	assert(out.state == JOY_ACTIVE);
	/* remote HOLD while waiting for neutral */
	in.enable_pin = false; steps(5); in.enable_pin = true; steps(50);
	assert(out.state == JOY_WAIT_NEUTRAL);
	in.remote_hold = true;
	steps(1);
	assert(out.state == JOY_STOPPED && (out.status_bits & JOY_S_REMOTE_STOP));
	puts("STOP ALWAYS WINS: remote HOLD / DISARM at once, no resume without release + neutral + re-grip: ok");
}

/* pascal's ruling: source-aware radio failsafe, window 3 s */
static void test_radio_failsafe_source(void)
{
	struct joy_rfs r = {};
	const uint32_t W = 3000;
	uint32_t t = 1000;

	/* no radio ever, occupant drives: never a radio failsafe */
	assert(!joy_rfs_allowed(&r, t, false, false, W));
	assert(!joy_rfs_allowed(&r, t + 60000, false, true, W));
	/* caregiver drives by radio, link lost: failsafe as today */
	t = 100000;
	assert(joy_rfs_allowed(&r, t, true, false, W));
	assert(joy_rfs_allowed(&r, t + 500, false, false, W));
	/* occupant grips at t+1000 with the radio still up, link lost 2 s later (within 3 s of the radio): acts */
	t += 10000;
	assert(joy_rfs_allowed(&r, t, true, false, W));
	assert(joy_rfs_allowed(&r, t + 1000, true, true, W));
	assert(joy_rfs_allowed(&r, t + 2000, false, true, W));
	assert(joy_rfs_allowed(&r, t + 3000, false, true, W));
	/* ... 3 s after the radio last commanded, the stick drives without it */
	assert(!joy_rfs_allowed(&r, t + 3001, false, true, W));
	assert(!joy_rfs_allowed(&r, t + 600000, false, true, W));
	/* radio present the whole time the occupant drives, then lost: no failsafe */
	t += 1000000;
	assert(joy_rfs_allowed(&r, t, true, false, W));
	for (uint32_t k = 1; k < 600; k++) {
		assert(joy_rfs_allowed(&r, t + k * 100, true, true, W) == (k * 100 <= W));
	}
	assert(!joy_rfs_allowed(&r, t + 60000, false, true, W));
	/* grip released with the radio gone: the radio is the source again -> acts (chair holds) */
	assert(joy_rfs_allowed(&r, t + 61000, false, false, W));
	/* re-grip, radio still gone, last radio command long ago: stick drives at once */
	assert(!joy_rfs_allowed(&r, t + 62000, false, true, W));
	/* window 0: only 'now' counts */
	struct joy_rfs z = {};
	assert(joy_rfs_allowed(&z, 5, true, false, 0));
	assert(joy_rfs_allowed(&z, 5, false, true, 0));
	assert(!joy_rfs_allowed(&z, 6, false, true, 0));
	puts("radio failsafe: acts only if the radio commanded now or within FS_RC_SRC_TIME; stick alone drives: ok");
}

int main(void)
{
	test_codec();
	test_neutral_rule();
	test_grip_release_no_resume();
	test_stale_and_fault();
	test_sequence();
	test_remote_stop_wins();
	test_radio_failsafe_source();
	printf("joy_arb: all tests passed (%d invariant checks)\n", checks);
	return 0;
}
