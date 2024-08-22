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
    RF
};

inline std::atomic<ControllerState> mcu_state;
inline std::atomic<ErrorState> error_state;

inline std::atomic<bool> new_state;

// ============================ Shared Variables =================================

// Task handles
inline TaskHandle_t controlTaskHandle = NULL;
inline TaskHandle_t attEstPredictTaskHandle = NULL;
inline TaskHandle_t attEstUpdateTaskHandle = NULL;
inline TaskHandle_t telemLoggerTaskHandle = NULL;
inline TaskHandle_t gyroCalibTaskHandle = NULL;

// semaphores

// Mutexes
inline SemaphoreHandle_t att_est_mutex = NULL;

// Sensors
inline Adafruit_LSM6DS3TRC imu;
// Adafruit_LSM6DSOX imu;
inline Adafruit_LIS3MDL mag;
inline Adafruit_BMP3XX bmp;

// State variables
inline UKF::Attitude att_estimator;

inline std::atomic<double> att_cmd_yaw = 0;
inline std::atomic<double> att_cmd_pitch = 0;
inline std::atomic<double> att_cmd_roll = 0;

inline std::atomic<double> tvc_cmd_x = 0;
inline std::atomic<double> tvc_cmd_y = 0;

inline sensor_msgs::BaroMsg baro_data;
inline sensor_msgs::AccelMsg accel_data;
inline sensor_msgs::GyroMsg gyro_data;
inline sensor_msgs::MagMsg mag_data;

inline std::atomic<double> lat = 0;
inline std::atomic<double> lon = 0;
inline std::atomic<double> gps_alt = 0;
inline std::atomic<double> gps_speed = 0;


inline std::atomic<int> pyro_1_cmd = 0;
inline std::atomic<int> pyro_2_cmd = 0;



#endif