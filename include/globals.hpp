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

#include <Wire.h>
#include "rtos_includes.hpp"
#include "attitude_estimator.hpp"
#include "captr_sensor_msgs.hpp"
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>

inline std::atomic<long long> msElapsed = 0;

// ================================= Constants ====================================

// ================================= State Machine ================================
enum class ControllerState
{
    LV_ON,
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
    BARO,
    MAG,
    RF,
    GPS
};

inline std::atomic<ControllerState> mcu_state_;
inline std::atomic<ErrorState> error_state_;

inline std::atomic<bool> new_state_;

// ============================ Shared Variables =================================

// semaphores

// Mutexes

// Sensors
inline Adafruit_LSM6DS3TRC imu__;
// Adafruit_LSM6DSOX imu__;
inline Adafruit_LIS3MDL mag__;
inline Adafruit_BMP3XX bmp__;
inline TinyGPSPlus gps__;

// State variables
inline UKF::Attitude att_estimator__;

inline std::atomic<double> att_cmd_yaw = 0;
inline std::atomic<double> att_cmd_pitch = 0;
inline std::atomic<double> att_cmd_roll = 0;

inline std::atomic<double> tvc_cmd_x = 0;
inline std::atomic<double> tvc_cmd_y = 0;

inline sensor_msgs::BaroMsg baro_data__;
inline sensor_msgs::AccelMsg accel_data__;
inline sensor_msgs::GyroMsg gyro_data__;
inline sensor_msgs::MagMsg mag_data__;

inline std::atomic<int> pyro_1_cmd = 0;
inline std::atomic<int> pyro_2_cmd = 0;



#endif