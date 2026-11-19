#pragma once

#include <cstdint>

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Zephyr_Namespace.h"

struct device;

class Zephyr::RCInput : public AP_HAL::RCInput {
public:
    RCInput();

    void    init() override;
    bool    new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t  read(uint16_t *periods, uint8_t len) override;
    const char *protocol() const override;
    bool priority_source_owns(uint8_t ch) override;
    bool priority_source_active(void) override { return _prio_active; }
    bool priority_source_new_input(void) override;

    /*
     * A local source with priority (the force joystick) on two channels (0-based, e.g. steering and throttle):
     * while active, read() serves its values on those channels, the receiver's on all others, and RC_Channel
     * ignores MAVLink overrides on those two. Values older than PRIO_STALE_MS read as the given neutral values
     * (a stalled timer thread must not leave a deflection standing). Timer thread.
     */
    void set_priority_input(uint8_t ch_a, uint16_t val_a, uint16_t neutral_a, uint8_t ch_b, uint16_t val_b,
                            uint16_t neutral_b, bool active);

    // registered as a timer proc; drains AP::RC and copies decoded
    // channels into the cache exposed to the main thread.
    void _timer_tick();

private:
    // Apply serial framing RC_CONFIGS[idx] to the UART and re-arm RX.
    void _apply_config(uint8_t idx);

    // resolved from ardupilot-rcinput DT alias; null if absent
    const struct device *_dev;
    bool                 _has_uart;
    uint8_t           _cfg_idx;
    uint32_t          _last_cfg_change_ms;
    uint32_t          _last_input_ms;
    bool              _detected;
    volatile uint32_t _current_baud;  // baud hint handed to process_byte()

    static constexpr uint8_t MAX_CH = 18u;
    volatile bool _prio_active;
    uint8_t       _prio_ch[2];
    uint16_t      _prio_val[2];
    uint16_t      _prio_neutral[2];
    uint32_t      _prio_ms;
    volatile bool _prio_new;
    static constexpr uint32_t PRIO_STALE_MS = 20u;
    uint16_t      _values[MAX_CH];
    uint8_t       _num_channels;
    volatile bool _new_input;

    static void _uart_isr(const struct device *dev, void *user_data);
};
