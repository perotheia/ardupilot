# AP_HAL_Zephyr host tests

Small standalone C++17 programs that test the pure helpers of the HAL on the
build host: no hardware, no Zephyr, no ArduPilot build, no test framework
(plain `assert`). The helpers are header-only files without Zephyr or
ArduPilot includes, and they are the code the firmware compiles:

| test | header | used by | checks |
|---|---|---|---|
| `test_cycle_clock.cpp` | `../CycleClock.h` | `system.cpp` (`AP_HAL::micros64()` with `CONFIG_AP_HAL_ZEPHYR_CYCLE_CLOCK`) | 64-bit extension of a fake 32-bit counter across wraps: exact cycle count, monotonic, microseconds equal to `floor(cycles * 1e6 / hz)` on every reading over simulated hours (no drift) at 19.2 MHz, 25 MHz, 200 MHz, 32768 Hz, 1 MHz; start with a non-zero counter and uptime |
| `test_delay_calc.cpp` | `../DelayCalc.h` | `Scheduler.cpp` (`delay_microseconds()` with `CONFIG_AP_HAL_ZEPHYR_DELAY_SLEEP_MIN_US`) | the number of tick edges to sleep, for every `us` 0..65535 and every phase 0..999 us within a tick, against Zephyr's wake-up rule: the wake-up is never after the deadline, also with ticks 3.1 % long; then the whole `delay_until()` loop against a simulated kernel: never early, no late sleep within the allowances, bounded final busy-wait, and a late sleep beyond them is reported |
| `test_mock_ins_sched.cpp` | `../MockINSSched.h` | `MockINS.cpp` | exactly 1000 samples/s on a fixed grid at 50 Hz, 400 Hz and 1 kHz call rates, with and without jitter; nothing due and no resynchronisation while the next sample is ahead of now (the unsigned underflow fixed in 244fafe); the catch-up clamp after a 100 ms stall |

Run:

    libraries/AP_HAL_Zephyr/tests/run.sh

It compiles every `test_*.cpp` with `$CXX` (default `g++`) at
`-std=c++17 -O2 -Wall -Wextra -Werror`, runs it and exits non-zero if one
fails. A test prints what it covered and `PASS`.

What these tests do not show: anything about the real kernel, timer driver
or board. In particular the delay test models the tick edges (nominal or
uniformly long ticks, a fixed wake-up latency); the real figures come from
the `dl=` field of the HAL monitor line on the board.
