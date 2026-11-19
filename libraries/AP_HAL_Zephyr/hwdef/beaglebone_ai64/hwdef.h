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

// empty token — SPIDevice.cpp always expands HAL_ZEPHYR_SPI_DEVICES
#define HAL_ZEPHYR_SPI_DEVICES

// no compass on this vehicle (GPS without a compass, skid steering)
#define HAL_MAG_PROBE_LIST

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

// Serial map (milestone 1):
//   SERIAL0 = rpmsg-tty  MAVLink2 to Linux (/dev/ttyRPMSGn), baud ignored
// later: GPS on UART6 (P9_14/P9_16), CRSF on UART4 (P9_37/P9_38)
#define DEFAULT_SERIAL0_PROTOCOL SerialProtocol_MAVLink2
#define DEFAULT_SERIAL0_BAUD 115

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
#else
#error "beaglebone_ai64: real IMUs (SPI6) are a later milestone; build with CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS"
#endif
