#include "MockINS.h"

#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS

#include <AP_HAL/AP_HAL.h>
#ifdef CONFIG_AP_HAL_ZEPHYR_MB_ALLOW_MOCK
#include <GCS_MAVLink/GCS.h>
#endif

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
    // which samples are due: MockINSSched.h (host-tested). After a stall at
    // most MOCK_MAX_CATCHUP periods are caught up, then the grid restarts.
    const auto due = _sample_clock.due(AP_HAL::micros64(), MOCK_SAMPLE_US, MOCK_MAX_CATCHUP);

    for (uint32_t i = 0; i < due.count; i++) {
        const uint64_t sample_us = due.first_us + i * MOCK_SAMPLE_US;
        _notify_new_accel_raw_sample(_accel_instance, Vector3f(0.0f, 0.0f, -GRAVITY_MSS), sample_us);
        _notify_new_gyro_raw_sample(_gyro_instance, Vector3f(0.0f, 0.0f, 0.0f), sample_us);
        ap_mock_ins_samples++;
    }
}

bool AP_InertialSensor_Zephyr_Mock::update()
{
#ifdef CONFIG_AP_HAL_ZEPHYR_MB_ALLOW_MOCK
    static uint32_t last_warn_ms;
    if (last_warn_ms == 0 || AP_HAL::millis() - last_warn_ms > 30000u) {
        last_warn_ms = AP_HAL::millis() | 1u;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BENCH: MOCK IMU (no real IMU found), motor power off");
    }
#endif
    accumulate();
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

#endif
