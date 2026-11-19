/*
 * Microsecond time from a free-running 32-bit hardware cycle counter.
 *
 * Pure C++ (no Zephyr, no ArduPilot includes): compiled into the firmware
 * through system.cpp and into the host tests in tests/.
 */
#pragma once

#include <stdint.h>

namespace Zephyr {

class CycleClock {
public:
    /*
      hz: counter frequency. counter_now: the counter at this moment.
      elapsed_us: the time this moment is to be reported as (the uptime so
      far, so that time keeps starting near 0 at boot).
     */
    void start(uint32_t hz, uint32_t counter_now, uint64_t elapsed_us)
    {
        uint32_t a = hz, b = 1000000U;
        while (b != 0) {
            const uint32_t t = a % b;
            a = b;
            b = t;
        }
        // us = cycles * _num / _den, reduced: 19.2 MHz -> 5 / 96
        _num = 1000000U / a;
        _den = hz / a;
        // the remainder term (r * _num, r < _den) fits 32 bits for every
        // sane counter: 32-bit division on the hot path
        _rem32 = (uint64_t)(_den - 1U) * _num <= UINT32_MAX;
        _last = counter_now;
        _cycles = us_to_cycles(elapsed_us);
        _started = true;
    }

    bool started() const { return _started; }

    /*
      64-bit extension: the cycles since the epoch. NOT reentrant: the caller
      serialises (irq_lock() on a single core). Correct as long as it is
      called at least once per counter wrap period (2^32 / hz seconds:
      223.7 s at 19.2 MHz): the wrap is taken from the modular difference to
      the previous reading.
     */
    uint64_t update(uint32_t counter_now)
    {
        _cycles += (uint32_t)(counter_now - _last);
        _last = counter_now;
        return _cycles;
    }

    /*
      floor(cycles * 1e6 / hz), exact for any ratio (19.2 cycles/us is not an
      integer) and without overflow or drift: quotient and remainder of the
      reduced fraction, never an accumulated rounded step.
     */
    uint64_t to_us(uint64_t cycles) const
    {
        const uint64_t q = cycles / _den;
        const uint32_t r = (uint32_t)(cycles % _den);
        if (_rem32) {
            return q * _num + (r * _num) / _den;
        }
        return q * _num + ((uint64_t)r * _num) / _den;
    }

    // floor(us * hz / 1e6), same method
    uint64_t us_to_cycles(uint64_t us) const
    {
        const uint64_t q = us / _num;
        const uint64_t r = us % _num;
        return q * _den + (r * _den) / _num;
    }

private:
    uint64_t _cycles = 0;
    uint32_t _last = 0;
    uint32_t _num = 1;
    uint32_t _den = 1;
    bool _rem32 = true;
    bool _started = false;
};

} // namespace Zephyr
