#!/usr/bin/env bash
# Host tests of the pure helpers of AP_HAL_Zephyr (no hardware, no Zephyr, no
# ArduPilot build): compiles each test_*.cpp with the host C++ compiler and
# runs it. See README.md.
#   libraries/AP_HAL_Zephyr/tests/run.sh
# Env: CXX (default g++), CXXFLAGS (added).
set -euo pipefail
cd "$(dirname "$0")"
CXX=${CXX:-g++}
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
fail=0
for src in test_*.cpp; do
    exe="$out/${src%.cpp}"
    "$CXX" -std=c++17 -O2 -Wall -Wextra -Werror ${CXXFLAGS:-} -o "$exe" "$src"
    if ! "$exe"; then
        echo "FAILED: $src" >&2
        fail=1
    fi
done
if [ "$fail" -ne 0 ]; then
    echo "AP_HAL_Zephyr host tests: FAILED" >&2
    exit 1
fi
echo "AP_HAL_Zephyr host tests: all passed"
