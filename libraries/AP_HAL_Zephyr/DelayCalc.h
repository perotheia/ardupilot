/*
 * Scheduler::delay_microseconds(): how long to sleep before the busy-wait.
 *
 * Pure C++ (no Zephyr, no ArduPilot includes): compiled into the firmware
 * through Scheduler.cpp and into the host tests in tests/.
 *
 * Zephyr wakes a sleeping thread only on a tick edge:
 *   k_sleep(K_TICKS(n))                  wakes on edge n + 1 after now
 *   k_sleep(K_TIMEOUT_ABS_TICKS(T + k))  wakes on edge k after now
 *                                        (T = k_uptime_ticks() now, k >= 1)
 * where edge 1 is the first tick edge after "now". With p = the time since
 * the last edge (0 <= p < tick) edge k is at k * tick - p from now: K_TICKS(n)
 * sleeps between n and n + 1 ticks, never less than n (kernel/timeout.c
 * z_add_timeout(): dticks = n + elapsed + 1; kernel/sleep.c).
 *
 * The functions here choose k so that edge k is not after the deadline, for
 * the worst case the caller cannot exclude:
 *   - p is unknown: the caller passes a lower bound (0 = just after an edge,
 *     the latest possible wake-up);
 *   - a tick may be longer than nominal (a timer driver that re-arms from
 *     "now" instead of from the previous edge): every tick is taken as up to
 *     1/DELAY_TICK_STRETCH_DIV (3.1 %) long;
 *   - DELAY_GUARD_US for the interrupt and the context switch at the edge.
 * What they cannot cover: threads of higher priority that run at the wake-up
 * edge, and ticks stretched by more than the allowance. Both make the
 * wake-up late; delay_until() returns at once then (late, never early).
 */
#pragma once

#include <stdint.h>

namespace Zephyr {

constexpr uint32_t DELAY_GUARD_US = 100;
constexpr uint32_t DELAY_TICK_STRETCH_DIV = 32;

/*
  The number of tick edges to sleep across (0 = do not sleep) with
  remaining_us to the deadline.

  phase_min_us: a lower bound of the time since the last tick edge (0 when
  unknown). tick_us: the nominal tick. min_edges: the smallest k the sleep
  primitive can do (1 with absolute timeouts, 2 with K_TICKS(k - 1): K_TICKS(0)
  only yields).

  Guarantee, for a true phase >= phase_min_us and ticks of nominal to
  (1 + 1/DELAY_TICK_STRETCH_DIV) nominal length: edge k is at least
  DELAY_GUARD_US before the deadline.
 */
inline uint32_t delay_sleep_edges(uint64_t remaining_us, uint32_t phase_min_us,
                                  uint32_t tick_us, uint32_t min_edges)
{
    if (tick_us == 0 || remaining_us <= DELAY_GUARD_US) {
        return 0;
    }
    if (phase_min_us >= tick_us) {
        phase_min_us = tick_us - 1;
    }
    // latest edge k: k * tick * (1 + 1/DIV) - phase <= remaining - guard
    const uint64_t budget = remaining_us - DELAY_GUARD_US + phase_min_us;
    const uint64_t k = (budget * DELAY_TICK_STRETCH_DIV) /
                       ((uint64_t)tick_us * (DELAY_TICK_STRETCH_DIV + 1U));
    if (k < min_edges || k == 0) {
        return 0;
    }
    return k > UINT32_MAX ? UINT32_MAX : (uint32_t)k;
}

/*
  Return when more than `us` microseconds have passed since t0_us on the
  port's clock (never early: the clock floors to 1 us, so one full
  microsecond more is awaited), sleeping while a whole tick fits.

  Port:
    uint64_t now_us();            microsecond clock
    void lock(); void unlock();   no preemption between now_us() and
                                  sleep_edges() (irq_lock(); the kernel
                                  releases it while the thread sleeps)
    void sleep_edges(uint32_t k); sleep until tick edge k after now; called
                                  locked, k >= min_edges
    void late(uint64_t us);       a sleep ended after the deadline (statistics)

  The phase is taken as unknown (0) on every pass. The first sleep therefore
  ends 1..2 ticks before the deadline; it ends on an edge, so the next pass
  can sleep one more edge where the primitive allows k = 1, and the final
  busy-wait is shorter than one stretched tick + DELAY_GUARD_US (two with
  min_edges = 2), plus whatever ran in between.
 */
template <class Port>
void delay_until(Port &port, uint64_t t0_us, uint32_t us, uint32_t tick_us, uint32_t min_edges)
{
    for (;;) {
        port.lock();
        const uint64_t elapsed = port.now_us() - t0_us;
        if (elapsed > us) {
            port.unlock();
            return;
        }
        const uint32_t k = delay_sleep_edges(us - elapsed, 0, tick_us, min_edges);
        if (k == 0) {
            port.unlock();
            break;
        }
        port.sleep_edges(k);
        const uint64_t woke = port.now_us() - t0_us;
        port.unlock();
        if (woke > us) {
            port.late(woke - us);
            return;
        }
    }
    while (port.now_us() - t0_us <= us) {
    }
}

} // namespace Zephyr
