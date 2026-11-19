/*
 * AI-64 camera IMU forwarder (CONFIG_AP_HAL_ZEPHYR_AI64_CAM_IMU): two ST LSM6DS3 on McSPI6 (left camera CS1,
 * right camera CS3, <= 4 MHz), read in FIFO batches by a low-priority thread, every sample stamped in J721E GTC
 * counts (the counter behind the A72 arch timer = Linux's clocksource), forwarded to Linux on the RPMsg endpoint
 * "imu-cam". Not part of ArduPilot's sensor set.
 *
 * Device setup: gyro + accel at 833 Hz (2000 dps, 16 g), FIFO continuous at 833 Hz with the timestamp counter
 * (25 us resolution) as the 4th data set, so a FIFO pattern is 9 words: gyro xyz, accel xyz, timestamp + steps.
 * Stamping (no interrupt latency involved): each batch reads the device's TIMESTAMP registers between two GTC
 * reads; the pairs (device ticks, GTC) give the device clock's offset and rate (a slow-filtered rate, nominally
 * 25 us = 5000 GTC counts at 200 MHz), and each FIFO sample's own timestamp is mapped through it. Expected
 * alignment: a few us (the SPI read of 3 bytes at 4 MHz takes ~10 us; the midpoint is used).
 * The INT1 pins are muxed (GPIO2) but not used: the thread polls every CONFIG_..._PERIOD_MS.
 *
 * Pairs with a GTC window > 100 us or off the model are dropped (the last good pair stays); nothing is stamped
 * before the first good pair. Health: implausible FIFO status 3 times, or WHO_AM_I / CTRL1_XL / FIFO_CTRL5 not as
 * set up (checked once a second) -> the device is set up again; while that fails, a LOST header goes out once a
 * second. The thread holds priority 6 (= ArduPilot's device work queue) while it holds SPI6.
 *
 * Packet (little endian): struct cam_imu_hdr, then n x struct cam_imu_sample.
 * Self-test at start: 'camimu: dev D (CSn) LSM6DS3|LSM6DS3TR-C|BMI160|ABSENT (0xXX)', GTC base/rate.
 * Register-level details (FIFO data set 4 byte order, TR-C timer bit) follow the datasheets: unverified on a
 * device yet.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/byteorder.h>

#include <openamp/open_amp.h>

#define LSM_WHO_AM_I     0x0f
#define LSM_WHO_AM_I_VAL 0x69 /* LSM6DS3 */
#define LSM_WHO_AM_I_TRC 0x6a /* LSM6DS3TR-C: same FIFO and timestamp layout, timer enable in CTRL10_C */
#define LSM_CTRL10_C     0x19
#define BMI160_CHIP_ID      0x00
#define BMI160_CHIP_ID_VAL  0xd1
#define LSM_FIFO_CTRL1   0x06
#define LSM_FIFO_CTRL2   0x07
#define LSM_FIFO_CTRL3   0x08
#define LSM_FIFO_CTRL4   0x09
#define LSM_FIFO_CTRL5   0x0a
#define LSM_CTRL1_XL     0x10
#define LSM_CTRL2_G      0x11
#define LSM_CTRL3_C      0x12
#define LSM_OUT_TEMP_L   0x20
#define LSM_FIFO_STATUS1 0x3a
#define LSM_FIFO_DATA_L  0x3e
#define LSM_TIMESTAMP0   0x40
#define LSM_TIMESTAMP2   0x42
#define LSM_TAP_CFG      0x58
#define LSM_WAKE_UP_DUR  0x5c

#define LSM_PATTERN_WORDS 9 /* gyro 3, accel 3, timestamp/step 3 */

/* J721E GTC (CFG1): 64-bit counter, its rate in CNTFID0 */
#define GTC_CFG1_BASE 0x00a90000u
#define GTC_CNTCV_LO  (GTC_CFG1_BASE + 0x08u)
#define GTC_CNTCV_HI  (GTC_CFG1_BASE + 0x0cu)
#define GTC_CNTFID0   (GTC_CFG1_BASE + 0x20u)

#define CAM_IMU_MAGIC 0x43554d49u /* 'IMUC' */

struct __packed cam_imu_hdr {
	uint32_t magic;
	uint8_t version; /* 1 */
	uint8_t dev;     /* 0 = left camera, 1 = right */
	uint8_t n;       /* samples that follow */
	uint8_t flags;   /* bit0 FIFO overrun, bit1 rate still nominal, bit2 temperature valid, bit3 device set up
			    again (clock restarted), bit4 device LOST (header only, n = 0, sent once a second) */
	uint32_t seq;    /* per device, +1 per data packet */
	uint32_t gtc_hz;
	uint16_t odr_hz;
	int16_t temp_cx100;      /* degC x 100 */
	uint16_t gyro_mdps_lsb;  /* 70 = 2000 dps */
	uint16_t accel_ug_lsb;   /* 488 = 16 g */
};

struct __packed cam_imu_sample {
	uint64_t gtc; /* sample time in GTC counts */
	int16_t gyro[3];
	int16_t accel[3];
};

#define CAM_IMU_MAX_SAMPLES ((496 - sizeof(struct cam_imu_hdr)) / sizeof(struct cam_imu_sample))

#define CAM_NODE_L DT_NODELABEL(cam_imu_l)
#define CAM_NODE_R DT_NODELABEL(cam_imu_r)
#define CAM_SPI_OP (SPI_OP_MODE_CONTROLLER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

struct cam_imu {
	struct spi_dt_spec spi;
	uint8_t id;
	bool present;
	uint8_t who; /* 0x69, 0x6a; BMI160 = 0xd1 (probe only) */
	uint32_t seq;
	/* device ticks -> GTC: the last pair and a filtered rate (GTC counts per tick, x 2^16) */
	bool calibrated;
	uint32_t pair_ticks;
	uint64_t pair_gtc;
	uint64_t rate_q16;
	uint32_t ref_ticks;
	uint64_t ref_gtc;
	uint32_t last_ticks;   /* for 24-bit unwrap */
	uint64_t ticks_hi;
	uint32_t batches;
	uint32_t overruns;
	bool rate_measured;     /* the rate comes from a >= 1 s baseline, not the nominal 25 us */
	bool lost;              /* was present, failed a health check: re-set up every second */
	bool reinit;            /* re-set up since the last packet (flag bit 3) */
	uint8_t fault_streak;
	uint32_t faults;
	uint32_t pairs_rejected;
	int64_t last_lost_report_ms;
};

static struct cam_imu cams[] = {
#if DT_NODE_HAS_STATUS_OKAY(CAM_NODE_L)
	{.spi = SPI_DT_SPEC_GET(CAM_NODE_L, CAM_SPI_OP), .id = DT_PROP(CAM_NODE_L, label_id)},
#endif
#if DT_NODE_HAS_STATUS_OKAY(CAM_NODE_R)
	{.spi = SPI_DT_SPEC_GET(CAM_NODE_R, CAM_SPI_OP), .id = DT_PROP(CAM_NODE_R, label_id)},
#endif
};

static struct rpmsg_endpoint cam_ept;
static atomic_t cam_ept_up;
static uint32_t gtc_hz;

uint64_t ai64_gtc_read(void)
{
	uint32_t hi, lo, hi2;

	do {
		hi = sys_read32(GTC_CNTCV_HI);
		lo = sys_read32(GTC_CNTCV_LO);
		hi2 = sys_read32(GTC_CNTCV_HI);
	} while (hi != hi2);
	return ((uint64_t)hi << 32) | lo;
}

/*
 * Priority ceiling while this thread holds SPI6: the Zephyr SPI lock is a semaphore (no priority inheritance), so
 * a low-priority holder preempted by the main (8) or UART (9) threads would block ArduPilot's own IMU reads on CS0
 * (device work queue, priority 6). Between transfers the thread runs at its own low priority.
 */
#define CAM_BUS_PRIO 6
static int cam_bus_enter(void)
{
	const int old = k_thread_priority_get(k_current_get());

	k_thread_priority_set(k_current_get(), CAM_BUS_PRIO);
	return old;
}

static void cam_bus_leave(int old)
{
	k_thread_priority_set(k_current_get(), old);
}

static int lsm_read(const struct cam_imu *c, uint8_t reg, uint8_t *buf, size_t len)
{
	uint8_t cmd = reg | 0x80u;
	const struct spi_buf tx_bufs[] = {{.buf = &cmd, .len = 1}};
	const struct spi_buf rx_bufs[] = {{.buf = NULL, .len = 1}, {.buf = buf, .len = len}};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};

	const int old = cam_bus_enter();
	const int rc = spi_transceive_dt(&c->spi, &tx, &rx);

	cam_bus_leave(old);
	return rc;
}

static int lsm_write(const struct cam_imu *c, uint8_t reg, uint8_t val)
{
	uint8_t b[2] = {reg & 0x7fu, val};
	const struct spi_buf tx_bufs[] = {{.buf = b, .len = 2}};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};

	const int old = cam_bus_enter();
	const int rc = spi_write_dt(&c->spi, &tx);

	cam_bus_leave(old);
	return rc;
}

/* write and read back (an SPI write alone proves nothing: an absent chip 'accepts' every write) */
static int lsm_write_check(const struct cam_imu *c, uint8_t reg, uint8_t val)
{
	uint8_t back = 0;

	if (lsm_write(c, reg, val) != 0 || lsm_read(c, reg, &back, 1) != 0) {
		return -EIO;
	}
	return back == val ? 0 : -EIO;
}

static const char *cam_imu_kind(uint8_t who)
{
	return who == LSM_WHO_AM_I_VAL ? "LSM6DS3" : who == LSM_WHO_AM_I_TRC ? "LSM6DS3TR-C" : "BMI160";
}

/*
 * Identity: ST parts answer WHO_AM_I (0x0f); a BMI160 answers CHIP_ID (0x00) once a first access (a dummy read
 * of 0x7f) has switched it to SPI. The breakout in use carries either footprint.
 */
static uint8_t cam_imu_probe(struct cam_imu *c, int *rc_out)
{
	uint8_t who = 0, id = 0, dummy;
	int rc = lsm_read(c, LSM_WHO_AM_I, &who, 1);

	*rc_out = rc;
	if (rc == 0 && (who == LSM_WHO_AM_I_VAL || who == LSM_WHO_AM_I_TRC)) {
		return who;
	}
	(void)lsm_read(c, 0x7f, &dummy, 1);
	k_busy_wait(100);
	if (lsm_read(c, BMI160_CHIP_ID, &id, 1) == 0 && id == BMI160_CHIP_ID_VAL) {
		return id;
	}
	printk("camimu: dev %u (CS%u) no known IMU: WHO_AM_I 0x%02x, BMI160 CHIP_ID 0x%02x\n", c->id,
	       c->spi.config.slave, who, id);
	return 0;
}

static bool lsm_setup(struct cam_imu *c)
{
	int rc;

	c->who = cam_imu_probe(c, &rc);
	printk("camimu: dev %u (CS%u) %s (0x%02x) rc %d\n", c->id, c->spi.config.slave,
	       c->who ? cam_imu_kind(c->who) : "ABSENT", c->who, rc);
	if (c->who == BMI160_CHIP_ID_VAL) {
		printk("camimu: dev %u is a BMI160: FIFO reader not implemented yet (probe only)\n", c->id);
		c->present = false;
		return false;
	}
	c->present = (c->who != 0);
	if (!c->present) {
		return false;
	}
	/* reset, then: block data update + register auto-increment */
	(void)lsm_write(c, LSM_CTRL3_C, 0x01);
	k_msleep(20);
	rc = lsm_write_check(c, LSM_CTRL3_C, 0x44);
	rc |= lsm_write_check(c, LSM_CTRL1_XL, 0x74);    /* 833 Hz, +-16 g */
	rc |= lsm_write_check(c, LSM_CTRL2_G, 0x7c);     /* 833 Hz, 2000 dps */
	rc |= lsm_write(c, LSM_WAKE_UP_DUR, 0x10); /* TIMER_HR: 25 us timestamp resolution */
	if (c->who == LSM_WHO_AM_I_TRC) {
		rc |= lsm_write(c, LSM_CTRL10_C, 0x20); /* TIMER_EN (TR-C) */
	} else {
		rc |= lsm_write(c, LSM_TAP_CFG, 0x80);  /* TIMER_EN (LSM6DS3) */
	}
	rc |= lsm_write(c, LSM_TIMESTAMP2, 0xaa);  /* timestamp reset */
	rc |= lsm_write(c, LSM_FIFO_CTRL1, 0x00);
	rc |= lsm_write(c, LSM_FIFO_CTRL2, 0x80);  /* TIMER_PEDO_FIFO_EN: timestamp as data set 4 */
	rc |= lsm_write(c, LSM_FIFO_CTRL3, 0x09);  /* gyro and accel into the FIFO, no decimation */
	rc |= lsm_write(c, LSM_FIFO_CTRL4, 0x08);  /* data set 4, no decimation */
	rc |= lsm_write_check(c, LSM_FIFO_CTRL5, 0x3e);  /* FIFO ODR 833 Hz, continuous mode */
	/* the timestamp restarted: the clock model starts over */
	c->calibrated = false;
	c->rate_measured = false;
	c->last_ticks = 0;
	c->ticks_hi = 0;
	c->fault_streak = 0;
	c->present = (rc == 0);
	if (!c->present) {
		printk("camimu: dev %u setup read-back failed\n", c->id);
	}
	return c->present;
}

/* 24-bit device ticks unwrapped to 64 bits (one batch per <= 400 s keeps it unambiguous) */
static uint64_t lsm_unwrap(struct cam_imu *c, uint32_t t24)
{
	if (t24 < c->last_ticks) {
		c->ticks_hi += 1u << 24;
	}
	c->last_ticks = t24;
	return c->ticks_hi | t24;
}

static uint64_t lsm_stamp(const struct cam_imu *c, uint32_t t24)
{
	/* signed 24-bit difference to the pair (the samples are a few ms older) */
	int32_t d = (int32_t)((t24 - c->pair_ticks) << 8) >> 8;

	return c->pair_gtc + (uint64_t)(((int64_t)d * (int64_t)c->rate_q16) >> 16);
}

/*
 * Read the device clock between two GTC reads: one (ticks, GTC) pair; update the rate model. A pair is only as
 * good as its GTC window: a read preempted or delayed on the shared bus (window > 100 us), or one that disagrees
 * with the model (a torn 24-bit read), is dropped and the last good pair stays. Returns whether the pair is used.
 */
static bool lsm_pair(struct cam_imu *c)
{
	uint8_t ts[3];
	const uint64_t g0 = ai64_gtc_read();

	if (lsm_read(c, LSM_TIMESTAMP0, ts, 3) != 0) {
		return false;
	}
	const uint64_t g1 = ai64_gtc_read();
	const uint32_t t24 = ts[0] | (ts[1] << 8) | ((uint32_t)ts[2] << 16);
	const uint64_t mid = g0 + (g1 - g0) / 2u;

	if (g1 - g0 > gtc_hz / 10000u) {
		c->pairs_rejected++;
		return false;
	}
	if (c->calibrated) {
		const uint64_t since = mid - c->pair_gtc;

		if (since > 2ull * gtc_hz) {
			/* no good pair for > 2 s: the model is too old to judge (and the 24-bit baseline may have
			   wrapped): re-base on this pair, keep the rate */
			c->pair_ticks = t24;
			c->pair_gtc = mid;
			c->ref_ticks = t24;
			c->ref_gtc = mid;
			return true;
		}
		/* tolerance: 100 us + the rate uncertainty over the time since the last pair (6 % nominal, 0.1 %
		   measured) */
		const int64_t err = (int64_t)(mid - lsm_stamp(c, t24));
		const int64_t tol = (int64_t)(gtc_hz / 10000u) + (int64_t)(since / (c->rate_measured ? 1000u : 16u));

		if (err > tol || err < -tol) {
			c->pairs_rejected++;
			return false;
		}
	}
	c->pair_ticks = t24;
	c->pair_gtc = mid;
	if (!c->calibrated) {
		c->rate_q16 = ((uint64_t)gtc_hz * 25u / 1000000u) << 16; /* nominal 25 us until measured */
		c->ref_ticks = t24;
		c->ref_gtc = mid;
		c->calibrated = true;
		return true;
	}
	/* rate over the baseline since the reference (re-based every ~10 s), low-pass once measured */
	const uint32_t dt = (t24 - c->ref_ticks) & 0xffffffu;

	if (dt > 40000u) { /* >= 1 s of baseline */
		const uint64_t meas = ((mid - c->ref_gtc) << 16) / dt;

		c->rate_q16 = c->rate_measured ? (c->rate_q16 * 7u + meas) / 8u : meas;
		c->rate_measured = true;
		if (dt > 400000u) {
			c->ref_ticks = t24;
			c->ref_gtc = mid;
		}
	}
	return true;
}

/* WHO_AM_I + configuration still as set up (a chip reset or a loose wire shows here) */
static bool lsm_healthy(struct cam_imu *c)
{
	uint8_t who = 0, ctrl1 = 0, fifo5 = 0;

	return lsm_read(c, LSM_WHO_AM_I, &who, 1) == 0 && who == c->who &&
	       lsm_read(c, LSM_CTRL1_XL, &ctrl1, 1) == 0 && ctrl1 == 0x74 &&
	       lsm_read(c, LSM_FIFO_CTRL5, &fifo5, 1) == 0 && fifo5 == 0x3e;
}

static void lsm_lost(struct cam_imu *c, const char *why)
{
	printk("camimu: dev %u %s: setting it up again\n", c->id, why);
	c->faults++;
	c->lost = !lsm_setup(c);
	c->reinit = true;
}

static void lsm_batch(struct cam_imu *c)
{
	static uint8_t pkt[496];
	struct cam_imu_hdr *h = (struct cam_imu_hdr *)pkt;
	struct cam_imu_sample *smp = (struct cam_imu_sample *)(pkt + sizeof(*h));
	uint8_t st[4], buf[LSM_PATTERN_WORDS * 2 * 4];
	uint32_t words, n = 0;

	/* once a second: still the chip and the configuration we set up? */
	if ((c->batches % 100u) == 99u && !lsm_healthy(c)) {
		lsm_lost(c, "failed the health check");
		return;
	}
	if (lsm_read(c, LSM_FIFO_STATUS1, st, 4) != 0) {
		return;
	}
	words = st[0] | ((st[1] & 0x0fu) << 8);
	const bool overrun = (st[1] & 0x40u) != 0;
	uint32_t pattern = st[2] | ((st[3] & 0x03u) << 8);
	const uint32_t depth = c->who == LSM_WHO_AM_I_TRC ? 2048u : 4096u;

	/* impossible status (an absent chip reads 0xff: words 4095, pattern 1023): no data from it */
	if (pattern >= LSM_PATTERN_WORDS || words > depth) {
		c->batches++;
		if (++c->fault_streak >= 3u) {
			lsm_lost(c, "FIFO status implausible 3 times");
		}
		return;
	}
	c->fault_streak = 0;
	(void)lsm_pair(c);
	if (!c->calibrated) {
		c->batches++;
		return; /* no good pair yet: nothing can be stamped (the FIFO keeps the samples) */
	}
	/* realign to the start of a pattern */
	while (pattern != 0 && words > 0) {
		uint8_t w[2];

		(void)lsm_read(c, LSM_FIFO_DATA_L, w, 2);
		words--;
		pattern = (pattern + 1u) % LSM_PATTERN_WORDS;
	}
	/* whole patterns, at most 4 per SPI transfer (a short bus hold for the other SPI6 users) */
	while (words >= LSM_PATTERN_WORDS && n < CAM_IMU_MAX_SAMPLES) {
		uint32_t groups = MIN(MIN(words / LSM_PATTERN_WORDS, 4u), CAM_IMU_MAX_SAMPLES - n);

		if (lsm_read(c, LSM_FIFO_DATA_L, buf, groups * LSM_PATTERN_WORDS * 2u) != 0) {
			break;
		}
		for (uint32_t g = 0; g < groups; g++, n++) {
			const uint8_t *p = buf + g * LSM_PATTERN_WORDS * 2u;
			/* data set 4: TIMESTAMP[15:8], TIMESTAMP[23:16], -, TIMESTAMP[7:0], step counter */
			uint32_t t24 = ((uint32_t)p[12] << 8) | ((uint32_t)p[13] << 16) | p[15];

			for (int a = 0; a < 3; a++) {
				smp[n].gyro[a] = (int16_t)sys_get_le16(p + 2 * a);
				smp[n].accel[a] = (int16_t)sys_get_le16(p + 6 + 2 * a);
			}
			smp[n].gtc = lsm_stamp(c, t24);
		}
		words -= groups * LSM_PATTERN_WORDS;
	}
	(void)lsm_unwrap(c, c->pair_ticks);

	h->magic = CAM_IMU_MAGIC;
	h->version = 1;
	h->dev = c->id;
	h->n = (uint8_t)n;
	/* flags: 1 FIFO overrun, 2 rate still nominal (stamps less exact), 4 temperature valid, 8 the device was set
	   up again since the last packet (its clock restarted) */
	h->flags = (overrun ? 1u : 0u) | (c->rate_measured ? 0u : 2u) | (c->reinit ? 8u : 0u);
	h->seq = c->seq;
	h->gtc_hz = gtc_hz;
	h->odr_hz = 833;
	h->temp_cx100 = 0;
	h->gyro_mdps_lsb = 70;
	h->accel_ug_lsb = 488;
	if ((c->batches++ % 100u) == 0u) {
		uint8_t t[2];

		if (lsm_read(c, LSM_OUT_TEMP_L, t, 2) == 0) {
			/* 16 LSB/degC (LSM6DS3), 256 LSB/degC (LSM6DS3TR-C), 0 = 25 degC */
			const int32_t lsb = c->who == LSM_WHO_AM_I_TRC ? 256 : 16;

			h->temp_cx100 = (int16_t)(2500 + ((int32_t)(int16_t)sys_get_le16(t) * 100) / lsb);
			h->flags |= 4u;
		}
	}
	if (overrun) {
		c->overruns++;
	}
	if (atomic_get(&cam_ept_up) && n > 0) {
		/* seq counts data packets only: a gap on Linux is a lost packet */
		if (rpmsg_trysend(&cam_ept, pkt, sizeof(*h) + n * sizeof(struct cam_imu_sample)) >= 0) {
			c->reinit = false;
		}
		c->seq++;
	}
}

static int cam_ept_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	ARG_UNUSED(priv);
	/* every message from Linux (re)binds the destination: a restarted reader gets a new endpoint address */
	ept->dest_addr = src;
	return RPMSG_SUCCESS;
}

/* rpmsg_remote.c: the RPMsg device is up */
void ap_rpmsg_board_link_up(struct rpmsg_device *rdev)
{
	if (rpmsg_create_ept(&cam_ept, rdev, "imu-cam", CONFIG_AP_HAL_ZEPHYR_AI64_CAM_IMU_ADDR, RPMSG_ADDR_ANY,
			     cam_ept_cb, NULL) == 0) {
		atomic_set(&cam_ept_up, 1);
	}
}

static void cam_imu_fn(void *a, void *b, void *d)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(d);
	bool any = false;

	gtc_hz = sys_read32(GTC_CNTFID0);
	printk("camimu: GTC at 0x%08x, CNTFID0 %u Hz, now %llu\n", GTC_CFG1_BASE, gtc_hz,
	       (unsigned long long)ai64_gtc_read());
	if (gtc_hz == 0u) {
		gtc_hz = 200000000u;
	}
	for (size_t i = 0; i < ARRAY_SIZE(cams); i++) {
		if (!spi_is_ready_dt(&cams[i].spi)) {
			printk("camimu: dev %u SPI not ready\n", cams[i].id);
			continue;
		}
		any |= lsm_setup(&cams[i]);
	}
	if (!any) {
		printk("camimu: no camera IMU answered: reader idle\n");
		return;
	}
	for (;;) {
		k_msleep(CONFIG_AP_HAL_ZEPHYR_AI64_CAM_IMU_PERIOD_MS);
		const int64_t now = k_uptime_get();

		for (size_t i = 0; i < ARRAY_SIZE(cams); i++) {
			struct cam_imu *c = &cams[i];

			if (c->present) {
				lsm_batch(c);
			} else if (c->lost && now - c->last_lost_report_ms >= 1000) {
				/* a device that was there: tell Linux once a second (header only, flag 16) and retry */
				struct cam_imu_hdr h = {.magic = CAM_IMU_MAGIC, .version = 1, .dev = c->id, .n = 0,
							.flags = 16u, .seq = c->seq, .gtc_hz = gtc_hz, .odr_hz = 833};

				c->last_lost_report_ms = now;
				if (atomic_get(&cam_ept_up)) {
					(void)rpmsg_trysend(&cam_ept, &h, sizeof(h));
				}
				c->lost = !lsm_setup(c);
				c->reinit = true;
			}
		}
	}
}

/* below ArduPilot's main thread (8) between transfers; at 6 while it holds SPI6 (cam_bus_enter) */
K_THREAD_DEFINE(cam_imu_tid, 2048, cam_imu_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 3000);
