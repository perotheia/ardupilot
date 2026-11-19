/*
 * ST LSM6DS3 / LSM6DS3TR-C 6-axis IMU, SPI or I2C. FIFO in continuous mode at 1666 Hz (gyro then accel, no
 * decimation: a FIFO pattern of 6 words), drained by a 1 kHz periodic callback.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM6DS3 : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    void start() override;
    bool update() override;

private:
    AP_InertialSensor_LSM6DS3(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum Rotation rotation);

    bool init();
    bool configure();
    void fifo_reset();
    void poll_data();
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t val, bool checked = false);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    enum Rotation rotation;
    uint8_t whoami;
    float temperature_degc;
    uint8_t temperature_counter;
    LowPassFilter2pFloat temperature_filter{100.0f, 1.0f};

    // 2000 dps: 70 mdps/LSB; 16 g: 0.488 mg/LSB (LSM6DS3 datasheet)
    static constexpr float gyro_scale = (70.0f / 1000.0f) * DEG_TO_RAD;
    static constexpr float accel_scale = (0.488f / 1000.0f) * GRAVITY_MSS;
    static constexpr uint16_t SAMPLE_RATE = 1666;
};
