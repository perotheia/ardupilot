#include "MockINS.h"

#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS

#include <AP_HAL/AP_HAL.h>

// BUS_TYPE_UNKNOWN so the mock can't collide with any real sensor ID
static const uint32_t MOCK_GYRO_ID  =
    AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UNKNOWN, 0, 0x01, AP_InertialSensor_Backend::DEVTYPE_GYR_MPU9250);
static const uint32_t MOCK_ACCEL_ID =
    AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UNKNOWN, 0, 0x01, AP_InertialSensor_Backend::DEVTYPE_ACC_MPU9250);

uint32_t ap_mock_ins_samples;

AP_InertialSensor_Zephyr_Mock::AP_InertialSensor_Zephyr_Mock(AP_InertialSensor &ins)
    : AP_InertialSensor_Backend(ins)
    , _accel_instance(0)
    , _gyro_instance(0)
    , _next_sample_us(0)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_Zephyr_Mock::probe(AP_InertialSensor &ins)
{
    AP_InertialSensor_Zephyr_Mock *backend =
        NEW_NOTHROW AP_InertialSensor_Zephyr_Mock(ins);
    if (!backend) {
        return nullptr;
    }

    // 1 kHz to match MPU-9250 default
    if (!ins.register_gyro(backend->_gyro_instance, 1000, MOCK_GYRO_ID) ||
        !ins.register_accel(backend->_accel_instance, 1000, MOCK_ACCEL_ID)) {
        delete backend;
        return nullptr;
    }

    return backend;
}

// Raw samples at the registered 1 kHz, synthesised from elapsed time. The
// frontend's wait_for_sample() (and the INS gyro calibration at boot) waits
// for the new-sample flags that only _notify_new_*_raw_sample() raises:
// publishing alone left the main loop spinning there forever.
void AP_InertialSensor_Zephyr_Mock::accumulate()
{
    const uint64_t now = AP_HAL::micros64();

    if (_next_sample_us == 0) {
        _next_sample_us = now;
    }
    // catch up at most 20 samples (20 ms) after a stall, then resync
    // (_next_sample_us is normally up to one period AHEAD of now: compare
    // only when it is behind, the difference is unsigned)
    if (now > _next_sample_us && now - _next_sample_us > 20 * MOCK_SAMPLE_US) {
        _next_sample_us = now - 20 * MOCK_SAMPLE_US;
    }
    while (_next_sample_us <= now) {
        _notify_new_accel_raw_sample(_accel_instance, Vector3f(0.0f, 0.0f, -GRAVITY_MSS),
                                     _next_sample_us);
        _notify_new_gyro_raw_sample(_gyro_instance, Vector3f(0.0f, 0.0f, 0.0f), _next_sample_us);
        _next_sample_us += MOCK_SAMPLE_US;
        ap_mock_ins_samples++;
    }
}

bool AP_InertialSensor_Zephyr_Mock::update()
{
    accumulate();
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

#endif
