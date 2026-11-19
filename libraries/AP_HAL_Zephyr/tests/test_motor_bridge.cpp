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
	.soft_max_ms = 1200,
	.is_idle_max_ma = 3000,
	.is_dead_duty = 5000,
	.is_dead_min_ma = 200,
	.is_fault_ms = 300,
	.vmot_max_mv = 27400,
	.vmot_hyst_mv = 400,
	.vmot_live_mv = 5000,
	.welded_ms = 500,
	.bus_min_mv = 18000,
	.coast_ms = 500,
	.estop_debounce_ms = 2,
	.i_filter_shift = 3,
	.duty_cap = 0,
	.max_gap_ms = 4,
};

static struct mb_state st;
static struct mb_in in;
static struct mb_out out, prev;
static int checks;
static bool latched_since_arm;   /* an e-stop / trip / gap happened while armed: no output until disarm */
static uint32_t release_edge_ms[MB_SIDES];
static bool was_release[MB_SIDES];

static int32_t sduty(const struct mb_out *o, int s)
{
	return (int32_t)o->pwm_fwd[s] - (int32_t)o->pwm_rev[s];
}

/* invariants that hold on every step */
static void invariants(void)
{
	for (int s = 0; s < MB_SIDES; s++) {
		assert(!(out.pwm_fwd[s] > 0 && out.pwm_rev[s] > 0));
		assert(out.pwm_fwd[s] <= MB_DUTY_MAX && out.pwm_rev[s] <= MB_DUTY_MAX);
		if (out.pwm_fwd[s] || out.pwm_rev[s]) {
			assert(out.brake_release[s] && out.enable[s]); /* never drive into a closed brake */
			/* the brake opened at least brake_release_ms before any duty */
			assert(was_release[s] && in.now_ms - release_edge_ms[s] >= cfg.brake_release_ms);
		}
		if (out.safe) {
			assert(!out.pwm_fwd[s] && !out.pwm_rev[s] && !out.enable[s] && !out.brake_release[s]);
		}
		/* |delta duty| per step bounded by the slew, except a hard stop to 0 */
		const int32_t d = sduty(&out, s) - sduty(&prev, s);
		if (sduty(&out, s) != 0) {
			assert(d <= (int32_t)cfg.soft_slew_per_ms && d >= -(int32_t)cfg.soft_slew_per_ms);
		}
		if (out.brake_release[s] && !was_release[s]) {
			release_edge_ms[s] = in.now_ms;
		}
		was_release[s] = out.brake_release[s];
	}
	if (out.safe) {
		assert(!out.drv_arm);
	}
	if (latched_since_arm) {
		assert(out.safe);
	}
	prev = out;
	checks++;
}

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
			in.last_cmd_ms[0] = in.last_cmd_ms[1] = in.now_ms;
		}
		step();
	}
}

static void reset_at(uint32_t t0)
{
	mb_init(&st);
	memset(&in, 0, sizeof(in));
	memset(&prev, 0, sizeof(prev));
	memset(was_release, 0, sizeof(was_release));
	latched_since_arm = false;
	in.now_ms = t0;
	in.vmot_mv = in.vbat_mv = 24000;
	in.sensors_ok = true;
	in.armed = true;
	in.last_cmd_ms[0] = in.last_cmd_ms[1] = in.now_ms;
}

static void reset(void)
{
	reset_at(1000);
}

static int32_t duty(int s)
{
	return sduty(&out, s);
}

static int spin_up(void)
{
	int t = 0;

	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	in.current_ma[0] = in.current_ma[1] = 5000;
	while ((duty(0) < MB_DUTY_MAX || duty(1) < MB_DUTY_MAX) && t < 2000) {
		steps(1, true);
		t++;
	}
	return t;
}

static void disarm_rearm(void)
{
	in.armed = false;
	latched_since_arm = false;
	steps(5, true);
	in.armed = true;
	steps(1, true);
}

static void test_ramp_and_brake(void)
{
	reset();
	steps(5, true);
	assert(!out.safe && !out.brake_release[0] && duty(0) == 0);
	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	in.current_ma[0] = in.current_ma[1] = 5000;
	steps(1, true);
	assert(out.brake_release[0] && duty(0) == 0);
	steps(98, true);
	assert(duty(0) == 0);
	int t = spin_up();
	assert(t >= 495 && t <= 510);
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

/* review C2: an e-stop released while still armed must not drive again until disarm + arm */
static void test_estop_cycle(void)
{
	reset();
	spin_up();
	in.estop_open = true;
	steps(1, true);
	assert(duty(0) == MB_DUTY_MAX || duty(0) == 0); /* debounce: at most one more step */
	steps(1, true);
	assert(out.safe && out.estop_latched);
	latched_since_arm = true;
	steps(10, true);
	in.estop_open = false; /* button released, vehicle still armed, stick still forward */
	steps(500, true);
	assert(out.safe && out.arm_block);
	disarm_rearm();
	assert(!out.safe && !out.estop_latched);
	steps(150, true);
	assert(duty(0) > 0); /* only now, through the brake release */
	/* a 1 ms glitch is filtered */
	in.estop_open = true;
	steps(1, true);
	in.estop_open = false;
	steps(5, true);
	assert(!out.estop_latched && duty(0) > 0);
	puts("e-stop: debounced, latched, no restart until disarm + arm: ok");
}

/* review C3: a late step is a hard latch; the next drive goes through the brake release and the ramp */
static void test_tick_gap(void)
{
	reset();
	spin_up();
	in.now_ms += 12;
	steps(1, true);
	assert(out.gap_event && out.safe);
	latched_since_arm = true;
	steps(100, true);
	disarm_rearm();
	steps(1, true);
	assert(duty(0) == 0 && out.brake_release[0]); /* release first, from 0 */
	steps(150, true);
	assert(duty(0) > 0 && duty(0) < MB_DUTY_MAX);
	puts("tick gap: hard latch, restart from 0 through the brake: ok");
}

/* review C4: a fault on one side stops both */
static void test_both_sides_stop(void)
{
	reset();
	spin_up();
	in.current_ma[0] = 50000;
	steps(10, true);
	assert(duty(0) == 0);                   /* tripped side: hard */
	assert(duty(1) > 0 && duty(1) < MB_DUTY_MAX); /* other side: soft ramp */
	steps(400, true);
	assert(duty(1) == 0);
	steps(600, true);
	assert(out.safe);
	in.current_ma[0] = 5000;
	disarm_rearm();
	assert(!out.arm_block);
	/* a fault while disarmed blocks arming while it lasts */
	reset();
	in.armed = false;
	in.current_ma[1] = 6000; /* current with the bridge off */
	steps(400, true);
	assert(out.arm_block);
	in.current_ma[1] = 0;
	steps(400, true);
	assert(!out.arm_block);
	puts("one side's fault stops both; faults block arming: ok");
}

static void test_trip_and_foldback(void)
{
	reset();
	spin_up();
	in.current_ma[0] = 30000;
	steps(60, true);
	assert(duty(0) < MB_DUTY_MAX && duty(0) > 0 && duty(1) == MB_DUTY_MAX);
	/* a chopped trip-level signal (every other ms) still trips with the leaky counter... */
	reset();
	spin_up();
	int t = 0;
	while (!out.trip_event[0] && t < 100) {
		in.current_ma[0] = (t % 3 != 2) ? 50000 : 0; /* 2 of 3 samples high */
		steps(1, true);
		t++;
	}
	assert(out.trip_event[0] && t < 40);
	puts("foldback + over-current trip (leaky, chopped IS): ok");
}

/* review M3: the soft stop coasts with the bridge enabled before the parking brake engages */
static void test_soft_watchdog(void)
{
	reset();
	spin_up();
	int t = 0;
	while (duty(0) != 0 && t < 2000) {
		steps(1, false);
		t++;
	}
	assert(t >= 100 + 290 && t <= 100 + 310);
	assert(!out.safe && out.enable[0] && out.brake_release[0]); /* coasting, short brake */
	steps((int)cfg.coast_ms - 5, false);
	assert(out.enable[0]);
	steps(10, false);
	assert(out.safe && !out.brake_release[0]);
	in.current_ma[0] = in.current_ma[1] = 0; /* stopped: no current */
	/* commands return with the stick still forward: no restart until it has been at neutral */
	steps(300, true);
	assert(duty(0) == 0);
	in.cmd[0] = in.cmd[1] = 0;
	steps(5, true);
	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	steps(150, true);
	assert(duty(0) > 0);
	puts("soft stop: watchdog ramp, coast, brake; restart only via neutral: ok");
}

static void test_soft_disarm(void)
{
	reset();
	spin_up();
	in.armed = false;
	steps(1, true);
	assert(duty(0) > 0 && duty(0) < MB_DUTY_MAX && out.brake_release[0]);
	steps(310 + (int)cfg.coast_ms, true);
	assert(out.safe);
	puts("soft stop: disarm while moving ramps and coasts: ok");
}

static void test_sense_faults(void)
{
	reset();
	spin_up();
	in.current_ma[1] = 0; /* no current at full duty */
	int t = 0;
	while (!out.sense_fault_event[1] && t < 1000) {
		steps(1, true);
		t++;
	}
	assert(t >= 299 && t <= 301);
	steps(1, true);
	assert(duty(0) > 0 && duty(0) < MB_DUTY_MAX && duty(1) > 0); /* both sides ramp */
	puts("current-sense plausibility (dead sensor) stops both softly: ok");
}

/* review M1: sensor loss or an implausible bus = no driving */
static void test_sensor_loss(void)
{
	reset();
	in.sensors_ok = false;
	in.cmd[0] = in.cmd[1] = MB_DUTY_MAX;
	steps(300, true);
	assert(duty(0) == 0 && !out.brake_release[0] && out.arm_block);
	reset();
	in.vbat_mv = in.vmot_mv = 0; /* readings of 0 V */
	in.cmd[0] = MB_DUTY_MAX;
	steps(300, true);
	assert(duty(0) == 0 && out.arm_block);
	reset();
	spin_up();
	in.sensors_ok = false; /* lost while driving: soft stop */
	steps(1, true);
	assert(duty(0) > 0 && duty(0) < MB_DUTY_MAX);
	puts("sensor loss / 0 V bus: no driving, soft stop while moving: ok");
}

static void test_overvolt(void)
{
	reset();
	in.vmot_mv = in.vbat_mv = 27200;
	in.cmd[0] = MB_DUTY_MAX;
	in.current_ma[0] = 5000;
	steps(300, true);
	assert(duty(0) == 0 && !out.brake_release[0] && out.overvolt);
	in.vmot_mv = in.vbat_mv = 26900;
	steps(1, true);
	assert(!out.overvolt);
	reset();
	spin_up();
	in.vbat_mv = 27600;
	steps(1, true);
	assert(out.overvolt && out.enable[0] && duty(0) == MB_DUTY_MAX - 20);
	steps(498, true);
	assert(duty(0) > 0 && out.enable[0]);
	steps(1, true);
	assert(duty(0) == 0 && out.enable[0]);
	in.vbat_mv = 27100;
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
	int rises = 0, p = 0;
	for (int i = 0; i < 100; i++) {
		steps(1, true);
		rises += (out.drv_arm && !p);
		p = out.drv_arm;
	}
	assert(rises >= 49 && rises <= 51);
	in.estop_open = true;
	steps(2, true);
	assert(!out.drv_arm && !out.welded_event); /* capacitors still charged: no report yet */
	steps((int)cfg.welded_ms, true);
	assert(out.welded_event || prev.welded_event);
	steps(20, true);
	assert(!out.welded_event && !out.drv_arm);
	puts("DRV_ARM square wave, welded contactor report after 0.5 s: ok");
}

/* random commands, faults and timings, also across the 32-bit ms wrap */
static void test_fuzz(void)
{
	srand(1);
	for (int run = 0; run < 200; run++) {
		reset_at(run % 2 ? UINT32_MAX - 2000u : 1000u);
		for (int i = 0; i < 5000; i++) {
			if (rand() % 50 == 0) {
				in.cmd[0] = rand() % (2 * MB_DUTY_MAX + 1) - MB_DUTY_MAX;
				in.cmd[1] = rand() % (2 * MB_DUTY_MAX + 1) - MB_DUTY_MAX;
			}
			if (rand() % 400 == 0) {
				in.armed = !in.armed;
				if (!in.armed) {
					latched_since_arm = false;
				}
			}
			in.estop_open = (rand() % 3000 == 0) ? !in.estop_open : in.estop_open;
			if (st.estop_latched && in.armed) {
				latched_since_arm = true;
			}
			in.current_ma[0] = (uint32_t)(rand() % 45000);
			in.current_ma[1] = (uint32_t)(rand() % 45000);
			in.vmot_mv = 23000 + (uint32_t)(rand() % 6000);
			in.vbat_mv = 23000 + (uint32_t)(rand() % 6000);
			in.sensors_ok = rand() % 500 != 0;
			if (rand() % 2000 == 0) {
				in.now_ms += 6; /* a late step */
			}
			steps(1, rand() % 20 != 0);
			if ((st.gap_latched || st.side[0].tripped || st.side[1].tripped) && in.armed) {
				latched_since_arm = true;
			}
		}
	}
	puts("fuzz (1M steps, also across the ms wrap): invariants hold: ok");
}

int main(void)
{
	test_ramp_and_brake();
	test_reversal();
	test_estop_cycle();
	test_tick_gap();
	test_both_sides_stop();
	test_trip_and_foldback();
	test_soft_watchdog();
	test_soft_disarm();
	test_sense_faults();
	test_sensor_loss();
	test_overvolt();
	test_drv_arm_wave_and_welded();
	test_fuzz();
	printf("motor_bridge: all tests passed (%d invariant checks)\n", checks);
	return 0;
}
