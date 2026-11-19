// Host test of ../CycleClock.h: the 64-bit extension of a 32-bit cycle
// counter and the cycles -> microseconds conversion behind AP_HAL::micros64().
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "../CycleClock.h"

using Zephyr::CycleClock;

static uint64_t rng_state = 0x9e3779b97f4a7c15ULL;
static uint64_t rng(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}

// exact reference
static uint64_t ref_us(unsigned __int128 cycles, uint32_t hz)
{
    return (uint64_t)(cycles * 1000000U / hz);
}

// Run a fake free-running 32-bit counter for sim_seconds, read at random
// intervals of up to max_step_s (below the wrap period), and compare every
// reading with the exact value.
static void run(uint32_t hz, uint32_t counter0, uint64_t elapsed_us, double sim_seconds,
                double max_step_s)
{
    CycleClock clk;
    clk.start(hz, counter0, elapsed_us);
    assert(clk.started());

    const unsigned __int128 cycles0 = clk.us_to_cycles(elapsed_us);
    // start() may round the epoch down by less than one microsecond
    assert(clk.to_us(clk.update(counter0)) <= elapsed_us);
    assert(elapsed_us - clk.to_us(clk.update(counter0)) <= 1);

    const uint64_t max_step = (uint64_t)(max_step_s * hz);
    assert(max_step < (1ULL << 32));
    const unsigned __int128 total_target = (unsigned __int128)(sim_seconds * hz);
    unsigned __int128 total = 0;    // true cycles since start()
    uint64_t prev_us = clk.to_us(clk.update(counter0));
    uint64_t wraps = 0, reads = 0;
    uint32_t counter = counter0;

    while (total < total_target) {
        // mostly short steps (1 kHz-like), sometimes long ones up to max_step
        uint64_t step = (rng() % 16 == 0) ? rng() % (max_step + 1) : rng() % (hz / 500 + 1);
        const uint32_t next = (uint32_t)(counter + step);
        if (next < counter) {
            wraps++;
        }
        counter = next;
        total += step;

        const uint64_t cyc = clk.update(counter);
        assert((unsigned __int128)cyc == cycles0 + total);     // exact extension
        const uint64_t us = clk.to_us(cyc);
        assert(us >= prev_us);                                 // monotonic
        assert(us == ref_us(cycles0 + total, hz));             // exact, no drift
        // a second reading of the same counter value changes nothing
        assert(clk.update(counter) == cyc);
        prev_us = us;
        reads++;
    }
    printf("  %9u Hz: %.0f s simulated, %llu reads, %llu counter wraps, last %llu us: exact\n",
           (unsigned)hz, sim_seconds, (unsigned long long)reads, (unsigned long long)wraps,
           (unsigned long long)prev_us);
    assert(wraps >= 2);
}

int main(void)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("test_cycle_clock\n");
    // the AI-64 DM timer: 19.2 MHz (19.2 cycles/us), wrap 223.7 s; 24 h
    run(19200000U, 0, 0, 24 * 3600.0, 200.0);
    // counter and uptime not zero at the first call, start just below a wrap
    run(19200000U, 0xfffffff0U, 123456789ULL, 3600.0, 200.0);
    run(25000000U, 0x80000000U, 5000, 6 * 3600.0, 150.0);      // wrap 171.8 s
    run(200000000U, 12345, 0, 2 * 3600.0, 20.0);               // wrap 21.5 s
    run(32768U, 0, 0, 96 * 3600.0, 3600.0);                    // 1e6/hz not an integer either
    run(1000000U, 0, 77, 3 * 3600.0, 4000.0);                  // 1 cycle = 1 us

    // conversion alone, far beyond any uptime (no overflow): 100 years
    {
        CycleClock clk;
        clk.start(19200000U, 0, 0);
        const unsigned __int128 cyc = (unsigned __int128)19200000U * 3600 * 24 * 365 * 100;
        assert(cyc < ((unsigned __int128)1 << 64));
        assert(clk.to_us((uint64_t)cyc) == ref_us(cyc, 19200000U));
        for (int i = 0; i < 1000000; i++) {
            const uint64_t c = rng() >> (rng() % 40);
            assert(clk.to_us(c) == ref_us(c, 19200000U));
        }
        // exact values: 96 cycles = 5 us
        assert(clk.to_us(95) == 4 && clk.to_us(96) == 5 && clk.to_us(19) == 0 && clk.to_us(20) == 1);
        printf("  conversion: 100 years and 1e6 random values exact at 19.2 MHz\n");
    }
    printf("test_cycle_clock: PASS\n");
    return 0;
}
