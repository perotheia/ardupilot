// Host test of ../DelayCalc.h: the sleep-then-busy-wait of
// Scheduler::delay_microseconds().
//
// Model of Zephyr's wake-up rule (kernel/timeout.c, kernel/sleep.c): a
// sleeping thread wakes only on a tick edge. Seen from a call made `phase`
// after the last edge:
//   k_sleep(K_TICKS(n))                 wakes on edge n + 1
//   k_sleep(K_TIMEOUT_ABS_TICKS(T + k)) wakes on edge k
// and edge j is at j * tick - phase when every tick has its nominal length,
// at j * (tick + stretch) - phase when ticks are long.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "../DelayCalc.h"

using namespace Zephyr;

static const uint32_t TICK = 1000;      // CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000
// the longest tick the calculation allows for
static const uint32_t STRETCH_MAX = TICK / DELAY_TICK_STRETCH_DIV;  // 31 us

// time from the call to tick edge j (j >= 1)
static int64_t edge_time(uint32_t j, uint32_t phase, uint32_t stretch)
{
    return (int64_t)j * (TICK + stretch) - phase;
}
// Zephyr's rule for a relative sleep of n ticks
static int64_t k_sleep_ticks_wake(uint32_t n, uint32_t phase, uint32_t stretch)
{
    return edge_time(n + 1, phase, stretch);
}

// (1) the pure calculation, dense: every us, every phase
static void test_calculation(void)
{
    uint64_t cases = 0;
    int64_t worst_margin = INT64_MAX;       // deadline - wake, over all sleeps
    int64_t worst_residual[3] = {0, 0, 0};  // busy-wait left, by min_edges

    for (uint32_t min_edges = 1; min_edges <= 2; min_edges++) {
        for (uint32_t us = 0; us <= 65535; us++) {
            for (uint32_t phase = 0; phase < TICK; phase++) {
                // phase unknown to the caller (what Scheduler.cpp passes)
                const uint32_t k = delay_sleep_edges(us, 0, TICK, min_edges);
                cases++;
                if (k == 0) {
                    // no sleep: only when not even min_edges stretched ticks fit
                    assert(us < DELAY_GUARD_US + (uint64_t)min_edges * (TICK + STRETCH_MAX) + 1);
                    if ((int64_t)us > worst_residual[min_edges]) {
                        worst_residual[min_edges] = us;
                    }
                    continue;
                }
                assert(k >= min_edges);
                // never after the deadline, for nominal and for the longest ticks,
                // through both sleep primitives
                const int64_t wake_nominal = edge_time(k, phase, 0);
                const int64_t wake_long = edge_time(k, phase, STRETCH_MAX);
                assert(k_sleep_ticks_wake(k - 1, phase, 0) == wake_nominal);
                assert(wake_long <= (int64_t)us - (int64_t)DELAY_GUARD_US);
                assert(wake_nominal <= wake_long);
                if ((int64_t)us - wake_long < worst_margin) {
                    worst_margin = (int64_t)us - wake_long;
                }
                // the residual busy-wait after ONE sleep is bounded: < 2 ticks
                // plus the allowance for long ticks plus the guard
                const int64_t residual = (int64_t)us - wake_nominal;
                assert(residual < 2 * (int64_t)TICK +
                                  ((int64_t)(k + 1) * TICK) / DELAY_TICK_STRETCH_DIV +
                                  DELAY_GUARD_US + 2);
            }
        }
    }
    // the caller knows the phase exactly: still never late, and the sleep is
    // never shorter than with the phase unknown
    for (uint32_t us = 0; us <= 65535; us += 3) {
        for (uint32_t phase = 0; phase < TICK; phase++) {
            const uint32_t k0 = delay_sleep_edges(us, 0, TICK, 1);
            const uint32_t k = delay_sleep_edges(us, phase, TICK, 1);
            cases++;
            assert(k >= k0);
            if (k != 0) {
                assert(edge_time(k, phase, STRETCH_MAX) <= (int64_t)us - (int64_t)DELAY_GUARD_US);
            }
        }
    }
    // degenerate arguments
    assert(delay_sleep_edges(0, 0, TICK, 1) == 0);
    assert(delay_sleep_edges(1000000, 0, 0, 1) == 0);
    assert(delay_sleep_edges(DELAY_GUARD_US, 999, TICK, 1) == 0);
    assert(delay_sleep_edges(5000, 5000, TICK, 1) == delay_sleep_edges(5000, TICK - 1, TICK, 1));
    assert(delay_sleep_edges(UINT64_MAX / 64, 0, 1, 1) == UINT32_MAX);

    printf("  calculation: %llu cases (us 0..65535 x phase 0..999 x min_edges 1,2; known phase):\n"
           "    wake-up never after the deadline; smallest margin %lld us with %u us long ticks;\n"
           "    longest wait without any sleep %lld us (abs timeouts) / %lld us (K_TICKS only)\n",
           (unsigned long long)cases, (long long)worst_margin, (unsigned)STRETCH_MAX,
           (long long)worst_residual[1], (long long)worst_residual[2]);
    assert(worst_margin >= (int64_t)DELAY_GUARD_US);
}

// (2) the whole delay_until() loop against a simulated kernel
struct SimPort {
    // configuration
    uint32_t stretch;       // every tick is TICK + stretch long
    uint32_t latency;       // from the edge to the thread running again
    uint32_t min_edges;
    // state
    uint64_t t;             // true time, us
    uint64_t edge0;         // time of an edge at or before the start
    bool locked = false;
    // observations
    uint32_t sleeps = 0;
    uint32_t late_calls = 0;
    uint64_t last_wake = 0;
    uint64_t polls = 0;

    uint64_t last_read = 0;
    uint64_t now_us()
    {
        polls++;
        last_read = t;
        return t++;         // every clock read costs 1 us: the busy-wait advances
    }
    void lock() { assert(!locked); locked = true; }
    void unlock() { assert(locked); locked = false; }
    void sleep_edges(uint32_t k)
    {
        assert(locked);
        assert(k >= min_edges);
        const uint64_t period = TICK + stretch;
        // edges are at edge0 + j * period: the first one strictly after t
        const uint64_t j = (t - edge0) / period + 1;
        t = edge0 + (j + k - 1) * period + latency;
        last_wake = t;
        sleeps++;
    }
    void late(uint64_t) { late_calls++; }
};

struct LoopStats {
    uint64_t runs = 0;
    uint64_t max_over = 0;          // return time - (t_call + us)
    uint64_t max_busy = 0;          // longest final busy-wait
    uint64_t late = 0;
    uint64_t slept_us = 0, total_us = 0;
};

static LoopStats test_loop(uint32_t stretch, uint32_t latency, uint32_t min_edges,
                           uint32_t us_step, uint32_t phase_step)
{
    LoopStats st;
    for (uint32_t us = 0; us <= 65535; us += us_step) {
        for (uint32_t phase = 0; phase < TICK + stretch; phase += phase_step) {
            SimPort port;
            port.stretch = stretch;
            port.latency = latency;
            port.min_edges = min_edges;
            const uint64_t t_call = 10000000ULL + us * 13ULL;
            port.edge0 = t_call - phase;
            port.t = t_call;

            // as Scheduler::delay_microseconds(): t0 is read, then the loop
            const uint64_t t0 = port.now_us();
            assert(t0 == t_call);
            delay_until(port, t0, us, TICK, min_edges);
            // the clock value the function returned on. The clock floors to
            // 1 us, so "at least us have really passed" needs a reading
            // strictly more than us after the first one.
            const uint64_t t_ret = port.last_read;
            assert(!port.locked);

            // never early
            assert(t_ret - t_call > us);
            const uint64_t over = t_ret - t_call - us;
            const uint64_t busy = port.sleeps ? t_ret - port.last_wake : t_ret - t_call;
            st.runs++;
            st.late += port.late_calls;
            if (over > st.max_over) {
                st.max_over = over;
            }
            if (busy > st.max_busy) {
                st.max_busy = busy;
            }
            st.total_us += t_ret - t_call;
            st.slept_us += (t_ret - t_call) - busy;
        }
    }
    return st;
}

int main(void)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("test_delay_calc\n");
    test_calculation();

    printf("  delay_until() against the simulated kernel (us 0..65535 step 7, phase step 13):\n");
    struct { uint32_t stretch, latency, min_edges; } cfg[] = {
        {0, 0, 1}, {0, 20, 1}, {0, DELAY_GUARD_US, 1},
        {10, 20, 1}, {STRETCH_MAX, 0, 1}, {STRETCH_MAX, DELAY_GUARD_US, 1},
        {0, 0, 2}, {STRETCH_MAX, DELAY_GUARD_US, 2},
    };
    for (const auto &c : cfg) {
        const LoopStats st = test_loop(c.stretch, c.latency, c.min_edges, 7, 13);
        printf("    tick +%2u us, wake latency %3u us, min_edges %u: %llu runs, late sleeps %llu, "
               "returns on a clock reading at most %llu us past t_call + us, longest busy-wait %llu us, "
               "asleep %.1f %% of the time\n",
               (unsigned)c.stretch, (unsigned)c.latency, (unsigned)c.min_edges,
               (unsigned long long)st.runs, (unsigned long long)st.late,
               (unsigned long long)st.max_over, (unsigned long long)st.max_busy,
               100.0 * (double)st.slept_us / (double)st.total_us);
        // within the allowances: no sleep ends after the deadline, and the
        // function returns on the first clock reading past it (us + 1; one
        // more when a pass of the loop read exactly us before)
        assert(st.late == 0);
        assert(st.max_over >= 1 && st.max_over <= 2);
        // final busy-wait: at most min_edges longest ticks + the guard (+ the
        // simulated clock reads; measured from the wake-up, so the latency
        // is not part of it)
        assert(st.max_busy <= (uint64_t)c.min_edges * (TICK + STRETCH_MAX + 1) + DELAY_GUARD_US + 4);
        assert(st.max_busy < 2 * TICK || c.min_edges == 2);
    }

    // beyond the allowances the sleep is late, and the function must say so
    // and return at once (late, never early)
    {
        const LoopStats st = test_loop(0, 5 * DELAY_GUARD_US, 1, 7, 13);
        printf("    wake latency %u us (beyond the %u us guard): %llu late sleeps of %llu runs, "
               "at most %llu us late\n", (unsigned)(5 * DELAY_GUARD_US), (unsigned)DELAY_GUARD_US,
               (unsigned long long)st.late, (unsigned long long)st.runs,
               (unsigned long long)st.max_over);
        assert(st.late > 0);
        assert(st.max_over <= 4 * DELAY_GUARD_US + 2);
    }
    printf("test_delay_calc: PASS\n");
    return 0;
}
