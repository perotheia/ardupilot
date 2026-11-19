/*
 * ST LSM6DS3 / LSM6DS3TR-C backend (see the header). Register map and FIFO from the LSM6DS3 datasheet
 * (DocID026899); the TR-C keeps the same registers for everything used here.
 *
 * FIFO: gyro and accel both at 1666 Hz into the FIFO (FIFO_CTRL3 = no decimation), FIFO ODR 1666 Hz,
 * continuous mode. The FIFO interleaves one gyro sample (3 words) and one accel sample (3 words); FIFO_STATUS3/4
 * give the position (0..5) of the next word, so a read always starts on a gyro word.
 */
#include "AP_InertialSensor_LSM6DS3.h"

#include <utility>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

#define LSM6DS3_FIFO_CTRL1   0x06
#define LSM6DS3_FIFO_CTRL2   0x07
#define LSM6DS3_FIFO_CTRL3   0x08
#define LSM6DS3_FIFO_CTRL4   0x09
#define LSM6DS3_FIFO_CTRL5   0x0a
#define LSM6DS3_WHO_AM_I     0x0f
#define LSM6DS3_CTRL1_XL     0x10
#define LSM6DS3_CTRL2_G      0x11
#define LSM6DS3_CTRL3_C      0x12
#define LSM6DS3_CTRL4_C      0x13
#define LSM6DS3_OUT_TEMP_L   0x20
#define LSM6DS3_FIFO_STATUS1 0x3a
#define LSM6DS3_FIFO_DATA_L  0x3e

#define LSM6DS3_ID     0x69
#define LSM6DS3TRC_ID  0x6a

#define CTRL3_C_SW_RESET 0x01
#define CTRL3_C_IF_INC   0x04
#define CTRL3_C_BDU      0x40
#define CTRL4_C_I2C_DIS  0x04

#define ODR_1666 0x8                          // ODR field value for 1.66 kHz (CTRL1_XL, CTRL2_G, FIFO_CTRL5)
#define CTRL1_XL_VAL ((ODR_1666 << 4) | 0x04) // +-16 g
#define CTRL2_G_VAL  ((ODR_1666 << 4) | 0x0c) // 2000 dps
#define FIFO_CTRL3_VAL 0x09                   // gyro and accel, no decimation
#define FIFO_CTRL5_CONT ((ODR_1666 << 3) | 0x06)
#define FIFO_CTRL5_BYPASS 0x00

#define PATTERN_WORDS 6
#define MAX_SETS_PER_READ 8

AP_InertialSensor_LSM6DS3::AP_InertialSensor_LSM6DS3(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                     enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DS3::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto *sensor = NEW_NOTHROW AP_InertialSensor_LSM6DS3(imu, std::move(dev), rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

uint8_t AP_InertialSensor_LSM6DS3::read_reg(uint8_t reg)
{
    uint8_t v = 0;
    dev->read_registers(reg, &v, 1);
    return v;
}

void AP_InertialSensor_LSM6DS3::write_reg(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}

bool AP_InertialSensor_LSM6DS3::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    whoami = read_reg(LSM6DS3_WHO_AM_I);
    if (whoami != LSM6DS3_ID && whoami != LSM6DS3TRC_ID) {
        dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        return false;
    }
    dev->setup_checked_registers(10, 20);
    bool ok = false;
    for (uint8_t tries = 0; tries < 5 && !ok; tries++) {
        ok = configure();
    }
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    if (!ok) {
        DEV_PRINTF("LSM6DS3: configuration failed\n");
    }
    return ok;
}

bool AP_InertialSensor_LSM6DS3::configure()
{
    write_reg(LSM6DS3_CTRL3_C, CTRL3_C_SW_RESET);
    hal.scheduler->delay(20);
    if (read_reg(LSM6DS3_CTRL3_C) & CTRL3_C_SW_RESET) {
        return false;
    }
    write_reg(LSM6DS3_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC, true);
    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        write_reg(LSM6DS3_CTRL4_C, CTRL4_C_I2C_DIS, true);
    }
    write_reg(LSM6DS3_CTRL1_XL, CTRL1_XL_VAL, true);
    write_reg(LSM6DS3_CTRL2_G, CTRL2_G_VAL, true);
    write_reg(LSM6DS3_FIFO_CTRL1, 0x00);
    write_reg(LSM6DS3_FIFO_CTRL2, 0x00, true);
    write_reg(LSM6DS3_FIFO_CTRL3, FIFO_CTRL3_VAL, true);
    write_reg(LSM6DS3_FIFO_CTRL4, 0x00, true);
    write_reg(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_CONT, true);
    hal.scheduler->delay(20);

    /* samples must be arriving */
    uint8_t st[2];
    if (!dev->read_registers(LSM6DS3_FIFO_STATUS1, st, 2)) {
        return false;
    }
    return (st[0] | ((st[1] & 0x0f) << 8)) > 0;
}

void AP_InertialSensor_LSM6DS3::fifo_reset()
{
    write_reg(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_BYPASS);
    write_reg(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_CONT);
}

void AP_InertialSensor_LSM6DS3::start()
{
    const uint8_t devtype = DEVTYPE_INS_LSM6DS3;
    if (!_imu.register_gyro(gyro_instance, SAMPLE_RATE, dev->get_bus_id_devtype(devtype)) ||
        !_imu.register_accel(accel_instance, SAMPLE_RATE, dev->get_bus_id_devtype(devtype))) {
        return;
    }
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        fifo_reset();
    }
    dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS3::poll_data, void));
}

void AP_InertialSensor_LSM6DS3::poll_data()
{
    uint8_t st[4];
    if (!dev->read_registers(LSM6DS3_FIFO_STATUS1, st, 4)) {
        _inc_gyro_error_count(gyro_instance);
        return;
    }
    uint16_t words = st[0] | ((st[1] & 0x0f) << 8);
    const bool overrun = (st[1] & 0x40) != 0;
    uint16_t pattern = st[2] | ((st[3] & 0x03) << 8);

    if (overrun || pattern >= PATTERN_WORDS) {
        /* lost samples (or a confused pattern): start clean; the frontend must not count the gap into the
           measured sample rate */
        fifo_reset();
        notify_accel_fifo_reset(accel_instance);
        notify_gyro_fifo_reset(gyro_instance);
        _inc_gyro_error_count(gyro_instance);
        _inc_accel_error_count(accel_instance);
        return;
    }
    /* realign on a gyro word */
    while (pattern != 0 && words > 0) {
        uint8_t w[2];
        if (!dev->read_registers(LSM6DS3_FIFO_DATA_L, w, 2)) {
            return;
        }
        words--;
        pattern = (pattern + 1) % PATTERN_WORDS;
    }

    uint8_t buf[MAX_SETS_PER_READ * PATTERN_WORDS * 2];
    while (words >= PATTERN_WORDS) {
        const uint16_t sets = MIN(words / PATTERN_WORDS, MAX_SETS_PER_READ);
        if (!dev->read_registers(LSM6DS3_FIFO_DATA_L, buf, sets * PATTERN_WORDS * 2)) {
            _inc_gyro_error_count(gyro_instance);
            return;
        }
        for (uint16_t s = 0; s < sets; s++) {
            const uint8_t *p = buf + s * PATTERN_WORDS * 2;
            /* chip axes (x, y, z up) -> the board frame ArduPilot's Invensense driver uses (y, x, -z): a chip
               mounted face up on the cape reads level with ROTATION_NONE, like the MPU-6050 beside it */
            const int16_t gx = int16_t(p[0] | (p[1] << 8)), gy = int16_t(p[2] | (p[3] << 8)), gz = int16_t(p[4] | (p[5] << 8));
            const int16_t ax = int16_t(p[6] | (p[7] << 8)), ay = int16_t(p[8] | (p[9] << 8)), az = int16_t(p[10] | (p[11] << 8));
            Vector3f gyro(gy, gx, -float(gz));
            Vector3f accel(ay, ax, -float(az));
            gyro *= gyro_scale;
            accel *= accel_scale;
            _rotate_and_correct_gyro(gyro_instance, gyro);
            _notify_new_gyro_raw_sample(gyro_instance, gyro);
            _rotate_and_correct_accel(accel_instance, accel);
            _notify_new_accel_raw_sample(accel_instance, accel);
        }
        words -= sets * PATTERN_WORDS;
    }

    if (++temperature_counter >= 10) {
        uint8_t t[2];
        if (dev->read_registers(LSM6DS3_OUT_TEMP_L, t, 2)) {
            /* LSM6DS3: 16 LSB/degC, TR-C: 256 LSB/degC, 0 = 25 degC */
            const float lsb_per_c = (whoami == LSM6DS3TRC_ID) ? 256.0f : 16.0f;
            temperature_degc = temperature_filter.apply(int16_t(t[0] | (t[1] << 8)) / lsb_per_c + 25.0f);
        }
        temperature_counter = 0;
    }

    AP_HAL::Device::checkreg reg;
    if (!dev->check_next_register(reg)) {
        log_register_change(dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

bool AP_InertialSensor_LSM6DS3::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);
    _publish_temperature(accel_instance, temperature_degc);
    return true;
}
