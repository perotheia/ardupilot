/*
 * Sample scheduling of the mock IMU (MockINS.cpp): which raw samples are due.
 *
 * Pure C++ (no Zephyr, no ArduPilot includes): compiled into the firmware
 * through MockINS.cpp and into the host tests in tests/.
 */
#pragma once

#include <stdint.h>

namespace Zephyr {

// the mock IMU: 1 kHz, as registered with the frontend
constexpr uint64_t MOCK_INS_SAMPLE_US = 1000;
// Periods caught up after a stall. Must be well above the longest regular
// gap between two accumulate() calls: one main loop period, 20 ms at
// Rover's 50 Hz. (It was 20: a 50 Hz caller with any jitter hit the clamp
// and lost samples.)
constexpr uint32_t MOCK_INS_MAX_CATCHUP = 50;

class MockSampleClock {
public:
    struct Due {
        uint64_t first_us;  // timestamp of the first due sample
        uint32_t count;     // samples due: first_us, first_us + period_us, ...
    };

    /*
      The samples due at now_us for a sample every period_us, and advance.
      Samples lie on a fixed grid from the first call: the long-run rate is
      exactly 1e6 / period_us whatever the call rate. After a stall at most
      max_catchup periods are caught up (max_catchup + 1 samples, the last
      one at now_us) and the grid restarts from there.
     */
    Due due(uint64_t now_us, uint64_t period_us, uint32_t max_catchup)
    {
        if (_next_us == 0) {
            _next_us = now_us;
        }
        // _next_us is normally up to one period AHEAD of now_us: compare only
        // when it is behind (the difference is unsigned; the unguarded
        // subtraction once resynchronised on every call)
        if (now_us > _next_us && now_us - _next_us > max_catchup * period_us) {
            _next_us = now_us - max_catchup * period_us;
        }
        if (_next_us > now_us || period_us == 0) {
            return Due{_next_us, 0};
        }
        const uint64_t count = (now_us - _next_us) / period_us + 1U;
        const Due d{_next_us, (uint32_t)count};
        _next_us += count * period_us;
        return d;
    }

    uint64_t next_us() const { return _next_us; }

private:
    uint64_t _next_us = 0;  // 0 = not started
};

} // namespace Zephyr
