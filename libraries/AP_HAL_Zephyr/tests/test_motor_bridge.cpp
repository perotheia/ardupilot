/* host tests for motor_bridge.c (the file the firmware compiles), run by run.sh */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../motor_bridge.c"

static const struct mb_cfg cfg = {
	.slew_per_ms = 20,        /* 0 -> 100 % in 500 ms */
	.reversal_dead_ms = 20,
	.brake_release_ms = 100,
	.brake_hold_ms = 500,
	.cmd_timeout_ms = 100,
	.i_limit_ma = 20000,
	.i_trip_ma = 40000,
	.trip_ms = 10,
	.soft_slew_per_ms = 34,   /* 100 % -> 0 in ~300 ms */
	.soft_max_ms = 600,
	.is_idle_max_ma = 3000,
	.is_dead_duty = 3000,
	.is_dead_min_ma = 200,
	.is_fault_ms = 50,
	.vmot_max_mv = 27400,
	.vmot_hyst_mv = 400,
	.vmot_live_mv = 5000,
};

static struct mb_state st;
static struct mb_in in;
static struct mb_out out;
static int checks;

/* invariants that hold on every step */
static void invariants(void)
{
	for (int s = 0; s < MB_SIDES; s++) {
		assert(!(out.pwm_fwd[s] > 0 && out.pwm_rev[s] > 0));
		assert(out.pwm_fwd[s] <= MB_DUTY_MAX && out.pwm_rev[s] <= MB_DUTY_MAX);
		if (out.pwm_fwd[s] || out.pwm_rev[s]) {
			assert(out.brake_release[s] && out.enable[s]); /* never drive into a closed brake */
		}
		if (out.safe) {
			assert(!out.pwm_fwd[s] && !out.pwm_rev[s] && !out.enable[s] && !out.brake_release[s]);
		}
	}
	if (out.safe) {
		assert(!out.drv_arm);
	}
	checks++;
}

/* one 1 ms step; current: plausible defaults unless a test sets them */
static void step(void)
{
	mb_step(&cfg, &st, &in, &out);
	invariants();
	in.now_ms++;
}

static void steps(int n, bool fresh)
{
	for (int i = 0; i < n; i++) {
		if (fresh) {
			in.last_cmd_ms = in.now_ms;
		}
		step();
	}
}

static void reset(void)
{
	mb_init(&st);
	memset(&in, 0, sizeof(in));
	in.now_ms = 1000;
	in.vmot_mv = 24000;
	in.vbat_mv = 24000;
	in.armed = true;
	in.last_cmd_ms = in.now_ms;
}

static int32_t duty(int s)
{
	return (int32_t)out.pwm_fwd[s] - (int32_t)out.pwm_rev[s];
}

/* drive both sides to full forward with plausible current; return the ms it took */
static int spin_up(void)
{
	int t = 0;

	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	in.current_ma[0] = in.current_ma[1] = 5000;
	while (duty(0) < MB_DUTY_MAX && t < 2000) {
		steps(1, true);
		t++;
	}
	return t;
}

static void test_ramp_and_brake(void)
{
	reset();
	steps(5, true); /* armed, idle: output stage armed, brakes applied, no duty */
	assert(!out.safe && !out.brake_release[0] && duty(0) == 0);
	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	in.current_ma[0] = in.current_ma[1] = 5000;
	steps(1, true);
	assert(out.brake_release[0] && duty(0) == 0); /* brake opens first */
	steps(98, true);
	assert(duty(0) == 0);                          /* still waiting for the brake */
	int t = spin_up();
	assert(t >= 495 && t <= 510);                  /* ramp ~500 ms after the release delay */
	/* back to 0: ramps down, then the brake closes after the hold time */
	in.cmd[0] = in.cmd[1] = 0;
	steps(500, true);
	assert(duty(0) == 0 && out.brake_release[0]);
	steps(510, true);
	assert(!out.brake_release[0] && !out.enable[0]);
	puts("ramp + brake sequence: ok");
}

static void test_reversal(void)
{
	reset();
	spin_up();
	in.cmd[0] = -MB_DUTY_MAX;
	int zero_at = -1, rev_at = -1;
	for (int i = 0; i < 2000 && rev_at < 0; i++) {
		steps(1, true);
		if (duty(0) == 0 && zero_at < 0) {
			zero_at = i;
		}
		if (duty(0) < 0) {
			rev_at = i;
		}
	}
	assert(zero_at > 0 && rev_at - zero_at >= (int)cfg.reversal_dead_ms);
	puts("reversal through 0 with dead time: ok");
}

static void test_hard_estop(void)
{
	reset();
	spin_up();
	in.estop_open = true;
	steps(1, true);
	assert(out.safe && duty(0) == 0 && out.estop_latched); /* immediate */
	in.estop_open = false;
	steps(50, true);
	assert(out.safe);                                       /* latched while armed */
	in.armed = false;
	steps(5, true);
	in.armed = true;
	steps(1, true);
	assert(!out.safe && !out.estop_latched);                /* fresh arm clears it */
	puts("hard stop: e-stop immediate, latched until re-arm: ok");
}

static void test_trip_and_foldback(void)
{
	reset();
	spin_up();
	in.current_ma[0] = 30000; /* above the limit: foldback */
	steps(50, true);
	assert(duty(0) < MB_DUTY_MAX && duty(0) > 0 && duty(1) == MB_DUTY_MAX);
	in.current_ma[0] = 50000; /* above the trip level */
	steps(9, true);
	assert(duty(0) > 0);
	steps(1, true);
	assert(out.trip_event[0] || duty(0) == 0);
	steps(1, true);
	assert(duty(0) == 0 && duty(1) == MB_DUTY_MAX); /* only that side, immediately */
	in.current_ma[0] = 0;
	steps(100, true);
	assert(duty(0) == 0);                           /* latched */
	puts("foldback + over-current trip latched per side: ok");
}

static void test_soft_watchdog(void)
{
	reset();
	spin_up();
	int t = 0;
	while (duty(0) != 0 && t < 2000) {
		steps(1, false); /* commands stop arriving */
		t++;
	}
	/* 100 ms watchdog, then ~300 ms ramp */
	assert(t >= 100 + 290 && t <= 100 + 310);
	assert(out.safe && !out.brake_release[0]);
	/* commands return: drives again */
	steps(1, true);
	assert(!out.safe);
	puts("soft stop: watchdog -> 0.3 s ramp -> brakes, resumes: ok");
}

static void test_soft_disarm(void)
{
	reset();
	spin_up();
	in.armed = false;
	steps(1, true);
	assert(duty(0) > 0 && duty(0) < MB_DUTY_MAX && out.brake_release[0]); /* ramping, not a cut */
	steps(310, true);
	assert(out.safe);
	puts("soft stop: disarm while moving ramps: ok");
}

static void test_sense_faults(void)
{
	reset();
	steps(5, true);
	in.current_ma[0] = 5000; /* current while the bridge is off */
	steps(49, true);
	assert(!out.sense_fault_event[0]);
	steps(1, true);
	assert(out.sense_fault_event[0]);
	in.current_ma[0] = 5000;
	in.cmd[0] = MB_DUTY_MAX;
	steps(800, true);
	assert(duty(0) == 0); /* latched */
	reset();
	spin_up();
	in.current_ma[1] = 0; /* no current at full duty */
	int t = 0;
	while (!out.sense_fault_event[1] && t < 200) {
		steps(1, true);
		t++;
	}
	assert(t >= 49 && t <= 51);
	steps(1, true);
	assert(duty(1) > 0 && duty(1) < MB_DUTY_MAX); /* a soft stop, ramping */
	puts("current-sense plausibility (idle current, dead sensor): ok");
}

static void test_overvolt(void)
{
	/* no driving before a reading below 27.0 V */
	reset();
	in.vmot_mv = in.vbat_mv = 27200;
	in.cmd[0] = MB_DUTY_MAX;
	in.current_ma[0] = 5000;
	steps(300, true);
	assert(duty(0) == 0 && !out.brake_release[0] && out.overvolt);
	in.vmot_mv = in.vbat_mv = 26900;
	steps(1, true);
	assert(!out.overvolt);
	/* above 27.4 V while driving (regeneration): ramp down at the normal slew, bridges kept enabled */
	reset();
	spin_up();
	in.vbat_mv = 27600; /* the battery side counts too */
	steps(1, true);
	assert(out.overvolt && out.enable[0] && duty(0) == MB_DUTY_MAX - 20);
	steps(498, true);
	assert(duty(0) > 0 && out.enable[0]);
	steps(1, true);
	assert(duty(0) == 0 && out.enable[0]);
	in.vbat_mv = 27100; /* inside the hysteresis band: still held */
	steps(10, true);
	assert(out.overvolt && duty(0) == 0);
	in.vbat_mv = 26900;
	steps(200, true);
	assert(!out.overvolt && duty(0) > 0);
	puts("over-voltage: enable below 27.0 V, gentle ramp above 27.4 V, no cut: ok");
}

static void test_drv_arm_wave_and_welded(void)
{
	reset();
	int rises = 0, prev = 0;
	for (int i = 0; i < 100; i++) {
		steps(1, true);
		rises += (out.drv_arm && !prev);
		prev = out.drv_arm;
	}
	assert(rises >= 49 && rises <= 51); /* a rising edge every 2 ms */
	in.estop_open = true;
	in.vmot_mv = 24000;
	steps(1, true);
	assert(out.welded_event && !out.drv_arm);
	steps(20, true);
	assert(!out.welded_event && !out.drv_arm); /* reported once, static low */
	puts("DRV_ARM square wave, welded contactor report: ok");
}

/* random commands, faults and timings: the invariants must hold on every step */
static void test_fuzz(void)
{
	srand(1);
	for (int run = 0; run < 200; run++) {
		reset();
		for (int i = 0; i < 5000; i++) {
			if (rand() % 50 == 0) {
				in.cmd[0] = rand() % (2 * MB_DUTY_MAX + 1) - MB_DUTY_MAX;
				in.cmd[1] = rand() % (2 * MB_DUTY_MAX + 1) - MB_DUTY_MAX;
			}
			if (rand() % 400 == 0) {
				in.armed = !in.armed;
			}
			in.estop_open = (rand() % 3000 == 0) ? !in.estop_open : in.estop_open;
			in.current_ma[0] = (uint32_t)(rand() % 45000);
			in.current_ma[1] = (uint32_t)(rand() % 45000);
			in.vmot_mv = 23000 + (uint32_t)(rand() % 6000);
			in.vbat_mv = 23000 + (uint32_t)(rand() % 6000);
			steps(1, rand() % 20 != 0);
		}
	}
	puts("fuzz (1M steps): invariants hold: ok");
}

int main(void)
{
	test_ramp_and_brake();
	test_reversal();
	test_hard_estop();
	test_trip_and_foldback();
	test_soft_watchdog();
	test_soft_disarm();
	test_sense_faults();
	test_overvolt();
	test_drv_arm_wave_and_welded();
	test_fuzz();
	printf("motor_bridge: all tests passed (%d invariant checks)\n", checks);
	return 0;
}
