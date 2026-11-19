// BeagleBone AI-64 (TI J721E), ArduRover on the MAIN R5FSS1 core0 beside
// Linux + TI vision_apps. Milestone 1: boots, scheduler runs, MAVLink2 on
// SERIAL0 over RPMsg (rpmsg-tty), mock IMU/baro, parameters in RAM.
#pragma once

#ifndef HAL_ZEPHYR_NUM_SERIAL_PORTS
#define HAL_ZEPHYR_NUM_SERIAL_PORTS 5
#endif
// SERIAL0 = the RPMsg bridge (no UART): MAVLink2 to the Linux host
#ifndef HAL_ZEPHYR_RPMSG_SERIAL_ENABLED
#define HAL_ZEPHYR_RPMSG_SERIAL_ENABLED 1
#endif
#define HAL_ZEPHYR_RPMSG_SERIAL_PORT 0

// SPI sensors: the cape IMU on McSPI6 CS0 (bus 0 = the ardupilot-spi0 alias, present in the imu-spi variant;
// without it the probe fails cleanly). {name, bus, cs, low Hz, high Hz, mode}
#define HAL_ZEPHYR_SPI_DEVICES \
    {"lsm6ds3_cape", 0, 0, 1000000, 10000000, 3},

// no compass on this vehicle (GPS without a compass, skid steering)
// compass: see the sensor section below

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif
#ifndef AP_MAVLINK_FTP_ENABLED
#define AP_MAVLINK_FTP_ENABLED 1
#endif
// @SYS/tasks.txt, @SYS/timers.txt, @SYS/threads.txt over MAVLink FTP: the
// scheduler timing report
#ifndef AP_FILESYSTEM_SYS_ENABLED
#define AP_FILESYSTEM_SYS_ENABLED 1
#endif
#ifndef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1
#endif
#ifndef AP_SIM_ENABLED
#define AP_SIM_ENABLED 0
#endif

// parameter defaults: the embedded region of AP_Param, filled after the link
// from hwdef/beaglebone_ai64/defaults.parm by Tools/scripts/apj_tool.py (and
// changeable on a binary without rebuilding: apj_tool.py zephyr.elf --set ...)
#define AP_PARAM_MAX_EMBEDDED_PARAM 2048

// RC from an ExpressLRS receiver in MAVLink mode (RADIO_RC_CHANNELS). The
// default gate (backends enabled, HAL_PROGRAM_SIZE_LIMIT_KB > 1024, GCS) is
// already true here (AP_HAL/board/zephyr.h sets the limit to 4096 KB): the
// define is redundant and kept explicit, because this board's RC depends on it
#define AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED 1
#define AP_GPS_UBLOX_ENABLED 1

// features without backing yet on this board
#define HAL_MOUNT_ENABLED 0
#define AP_WINCH_ENABLED 0
#define AP_GPS_SBF_ENABLED 0
#define HAL_LOGGING_ENABLED 0
#define AP_NOTIFY_TOSHIBALED_ENABLED 0
#define AP_NOTIFY_NCP5623_ENABLED 0
#define AP_NOTIFY_LP5562_ENABLED 0
#define AP_NOTIFY_IS31FL3195_ENABLED 0

#undef ARRAY_SIZE
#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

#include <zephyr/sys/cbprintf.h>

#ifdef __cplusplus
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Math/AP_Math.h>
#include <AP_Mission/AP_Mission.h>
#endif

#undef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

// Serial map:
//   SERIAL0 = rpmsg-tty  MAVLink2 to Linux (/dev/ttyRPMSGn), baud ignored
//   SERIAL1 = UART8      P8_28 RX / P8_29 TX: ExpressLRS in MAVLink mode
//                        (RC + telemetry), 460800
//   SERIAL3 = UART6      P9_14 RX / P9_16 TX: u-blox GPS (autobaud; 38400 to
//                        start)
// P9_33 and P9_35..P9_40 are analog-only 1.8 V pins (MCU_ADC0) and must never
// be muxed to a digital function: no UART4 on P9_37/P9_38.
#define DEFAULT_SERIAL0_PROTOCOL SerialProtocol_MAVLink2
#define DEFAULT_SERIAL0_BAUD 115
#define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
#define DEFAULT_SERIAL1_BAUD 460
#define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
#define DEFAULT_SERIAL3_BAUD 38

// sensors: after the feature gates and the AP_Math dance (MockINS.h pulls the
// AP_InertialSensor config chain, which defaults every gate it meets)
#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS
#ifdef __cplusplus
#include <AP_HAL_Zephyr/MockINS.h>
#include <AP_Baro/AP_Baro_Dummy.h>
#endif
#define HAL_INS_PROBE_LIST \
    ADD_BACKEND(AP_InertialSensor_Zephyr_Mock::probe(*this))
#define HAL_BARO_PROBE_LIST \
    _add_backend(AP_Baro_Dummy::probe(*this))
#define HAL_MAG_PROBE_LIST
#else
/*
 * Real sensors on MAIN I2C2 (P9_19 SCL / P9_20 SDA, 400 kHz, the overlay's ardupilot-i2c0): IMU 1 = MPU-6050
 * at 0x68 (AP_InertialSensor_Invensense, 1 kHz with its FIFO), compass = LIS3MDL at 0x1C (the Adafruit 9-DoF
 * board; external). No barometer on this vehicle (dummy). Mounting: ROTATION_NONE here; set
 * AHRS_ORIENTATION / COMPASS_ORIENT on the vehicle. The second IMU comes later on SPI6.
 */
#ifdef __cplusplus
#include <AP_Baro/AP_Baro_Dummy.h>
#include <AP_InertialSensor/AP_InertialSensor_LSM6DS3.h>
#endif
// IMU 1 = the cape LSM6DS3 on SPI6 CS0 (variant imu-spi), IMU 2 = the MPU-6050 on I2C2 (variant imu); an
// absent sensor's probe just fails
#define HAL_INS_PROBE_LIST \
    ADD_BACKEND(AP_InertialSensor_LSM6DS3::probe(*this, hal.spi->get_device("lsm6ds3_cape"), ROTATION_NONE)); \
    ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.i2c_mgr->get_device(0, 0x68, 400000), ROTATION_NONE))
#define HAL_BARO_PROBE_LIST \
    _add_backend(AP_Baro_Dummy::probe(*this))
#define HAL_MAG_PROBE_LIST \
    { add_backend(DRIVER_LIS3MDL, AP_Compass_LIS3MDL::probe(hal.i2c_mgr->get_device(0, 0x1c, 400000), \
                                                          true, ROTATION_NONE)); RETURN_IF_NO_SPACE; }
#endif
