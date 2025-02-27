/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: main.cpp
Auth: Alex Wang, Yubo Wang
Desc: Global variables and constants header file to be included in all relevant files. 
      Variable definitions use 'inline' to avoid linker errors.

*/

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "config.hpp"

#include <Wire.h>
#include "rtos_includes.hpp"
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>

#include "attitude_estimator.hpp"
#include "captr_sensor_msgs.hpp"
#include "CAPTR_PIN_DRIVER.hpp"
#include "mount_lib.hpp"

// #include <driver_bmp390.h>

inline uint32_t msElapsed = 0;

// ================================= Constants ====================================

// ================================= State Machine ================================
enum class ControllerState
{
    STBY,
    CALIBRATING,
    LAUNCH_DETECT,
    POWERED_ASCENT,
    COAST,
    RECOVERY,
    // POWERED_DESCENT,
    LANDED,
};

enum class ErrorState
{
    NONE,

    GENERAL, 

    FSM,               

    CONTROL,

    ATT_EST,

    LOGGING,

    FLASH_INIT,
    FLASH_CREATE,
    FLASH_WRITE,

    ACCEL, 
    GYRO,
    IMU,
    BARO,
    MAG,
    RF,
    GPS
};

inline std::atomic<ControllerState> currentState;
inline std::atomic<ErrorState> errorState_;

inline std::atomic<bool> new_state_;

// ============================ Shared Variables =================================

//----------------------------------------semaphores

//----------------------------------------Mutexes
inline SemaphoreHandle_t serial_port_mutex__;

//----------------------------------------Sensors
inline Adafruit_LSM6DSO32 imu__;
// Adafruit_LSM6DSOX imu__;
inline Adafruit_LIS3MDL mag__;
inline Adafruit_BMP3XX bmp__;
// inline bmp390_handle_s bmp__;
inline TinyGPSPlus gps__;

//----------------------------------------State variables
inline UKF::Attitude att_estimator__;

inline tvc_mount_lib::TVC_mount tvc_mount__(SERVO_PITCH_PIN, SCALING_PITCH, OFFSET_PITCH, LIMIT_PITCH, SERVO_YAW_PIN, SCALING_YAW, OFFSET_YAW, LIMIT_YAW);

inline float ground_altitude_offset_msl__ = 0.0;
inline sensor_msgs::BaroMsg baro_data__;
inline sensor_msgs::AccelMsg accel_data__;
inline sensor_msgs::GyroMsg gyro_data__;
inline sensor_msgs::MagMsg mag_data__;

inline std::atomic<int> pyro_1_cmd = 0;
inline std::atomic<int> pyro_2_cmd = 0;

inline std::atomic<float> gyro_offset_x = 0;
inline std::atomic<float> gyro_offset_y = 0;
inline std::atomic<float> gyro_offset_z = 0;

#endif