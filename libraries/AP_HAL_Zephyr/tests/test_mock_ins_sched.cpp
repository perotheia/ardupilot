// Host test of ../MockINSSched.h: the sample scheduling of the mock IMU.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "../MockINSSched.h"

using Zephyr::MockSampleClock;

// the values MockINS.cpp runs with
static const uint64_t PERIOD = Zephyr::MOCK_INS_SAMPLE_US;
static const uint32_t CATCHUP = Zephyr::MOCK_INS_MAX_CATCHUP;

// Call due() at call_hz for `seconds` (with a deterministic jitter of up to
// jitter_us on every call time) and check: timestamps on the 1 ms grid,
// strictly increasing, never in the future, exactly 1000 samples per second.
static void rate(uint32_t call_hz, uint32_t seconds, uint32_t jitter_us)
{
    MockSampleClock clk;
    const uint64_t t0 = 5000123;        // not on a round number
    const uint64_t call_period = 1000000U / call_hz;
    uint64_t total = 0, last_ts = 0;
    uint32_t max_per_call = 0;
    uint64_t per_second[2] = {0, 0};

    for (uint64_t k = 0; k <= (uint64_t)seconds * call_hz; k++) {
        const uint64_t now = t0 + k * call_period + (k * 7919U) % (jitter_us + 1U);
        const auto d = clk.due(now, PERIOD, CATCHUP);
        for (uint32_t i = 0; i < d.count; i++) {
            const uint64_t ts = d.first_us + i * PERIOD;
            assert((ts - t0) % PERIOD == 0);            // on the grid of the first call
            assert(total == 0 || ts == last_ts + PERIOD); // none lost, none doubled
            assert(ts <= now);                          // never from the future
            last_ts = ts;
            total++;
            if (ts - t0 < 2000000U) {
                per_second[(ts - t0) / 1000000U]++;
            }
        }
        if (d.count > max_per_call) {
            max_per_call = d.count;
        }
        assert(clk.next_us() > now);                    // next sample is ahead
        assert(clk.next_us() - now <= PERIOD);
    }
    // samples at t0, t0 + 1 ms, ... up to the last call time
    const uint64_t expect = (uint64_t)seconds * 1000U + 1U;
    printf("  %3u Hz calls, jitter %3u us, %u s: %llu samples (expected %llu), "
           "max %u per call\n", (unsigned)call_hz, (unsigned)jitter_us, (unsigned)seconds,
           (unsigned long long)total, (unsigned long long)expect, (unsigned)max_per_call);
    assert(total == expect || total == expect + jitter_us / PERIOD);
    assert(per_second[0] == 1000 && per_second[1] == 1000);
    assert(max_per_call <= call_period / PERIOD + 1U + jitter_us / PERIOD);
}

int main(void)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("test_mock_ins_sched\n");
    rate(50, 60, 0);
    rate(400, 60, 0);
    rate(50, 60, 900);
    rate(400, 60, 900);
    rate(1000, 10, 0);

    // regression (commit 244fafe): next > now must not be read as "far
    // behind". Called again before the next sample is due: nothing due and
    // the schedule does not move.
    {
        MockSampleClock clk;
        auto d = clk.due(10000, PERIOD, CATCHUP);
        assert(d.count == 1 && d.first_us == 10000 && clk.next_us() == 11000);
        for (uint64_t now = 10000; now < 11000; now += 37) {
            d = clk.due(now, PERIOD, CATCHUP);
            assert(d.count == 0);
            assert(clk.next_us() == 11000);     // the underflow bug moved this to now - 20 ms
        }
        // and time going backwards a little does no harm either
        d = clk.due(9990, PERIOD, CATCHUP);
        assert(d.count == 0 && clk.next_us() == 11000);
        d = clk.due(11000, PERIOD, CATCHUP);
        assert(d.count == 1 && d.first_us == 11000);
        printf("  next > now: nothing due, schedule unchanged (underflow regression)\n");
    }

    // catch-up clamp: a 100 ms stall gives CATCHUP + 1 samples (the CATCHUP
    // periods before now and the one at now), not 100, and the grid restarts
    // at now
    {
        assert(CATCHUP < 100);
        MockSampleClock clk;
        auto d = clk.due(50000, PERIOD, CATCHUP);
        assert(d.count == 1);
        const uint64_t now = 50000 + 100000 + 321;
        d = clk.due(now, PERIOD, CATCHUP);
        assert(d.count == CATCHUP + 1);
        assert(d.first_us == now - CATCHUP * PERIOD);
        assert(d.first_us + (d.count - 1) * PERIOD == now);
        assert(clk.next_us() == now + PERIOD);
        // a stall just inside the limit loses nothing
        MockSampleClock clk2;
        (void)clk2.due(50000, PERIOD, CATCHUP);
        d = clk2.due(50000 + CATCHUP * PERIOD + 999, PERIOD, CATCHUP);
        assert(d.count == CATCHUP && d.first_us == 51000);
        printf("  100 ms stall: %u samples, resynchronised; %u.999 ms stall: %u samples, none lost\n",
               (unsigned)(CATCHUP + 1), (unsigned)CATCHUP, (unsigned)CATCHUP);
    }
    // the reason the clamp is not 20: one 50 Hz period that comes 0.9 ms
    // late (20.9 ms after a call 0.1 ms before a sample) already passes a
    // 20 ms clamp, which moves the sample grid; the production value does not
    {
        MockSampleClock c20, cprod;
        (void)c20.due(1000, PERIOD, 20);
        (void)cprod.due(1000, PERIOD, CATCHUP);
        (void)c20.due(1900, PERIOD, 20);
        (void)cprod.due(1900, PERIOD, CATCHUP);
        const auto d20 = c20.due(1900 + 20900, PERIOD, 20);
        const auto dprod = cprod.due(1900 + 20900, PERIOD, CATCHUP);
        assert(d20.first_us % PERIOD != 0);     // grid lost
        assert(dprod.first_us == 2000 && dprod.count == 21 && cprod.next_us() == 23000);
        printf("  50 Hz period 0.9 ms late: grid kept with the clamp at %u (moved with 20)\n",
               (unsigned)CATCHUP);
    }
    printf("test_mock_ins_sched: PASS\n");
    return 0;
}
