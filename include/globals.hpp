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

#include "rtos_includes.hpp"

inline std::atomic<long long> msElapsed = 0;

// ================================= Constants ====================================

// ================================= State Machine ================================
enum class ControllerState
{
    LV_ON,
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

    IMU, 
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

// semaphores

// Mutexes
inline SemaphoreHandle_t att_est_mutex = NULL;

// State variables
inline UKF::Attitude att_estimator;

inline std::atomic<double> att_cmd_yaw = 0;
inline std::atomic<double> att_cmd_pitch = 0;
inline std::atomic<double> att_cmd_roll = 0;

inline std::atomic<double> tvc_cmd_x = 0;
inline std::atomic<double> tvc_cmd_y = 0;

inline std::atomic<double> altitude_agl = 0;
inline std::atomic<double> altitude_msl = 0;
inline std::atomic<double> baro_Pa = 0;

inline std::atomic<double> accel_x = 0;
inline std::atomic<double> accel_y = 0;
inline std::atomic<double> accel_z = 0;

inline std::atomic<double> gyro_x = 0;
inline std::atomic<double> gyro_y = 0;
inline std::atomic<double> gyro_z = 0;

inline std::atomic<double> mag_x = 0;
inline std::atomic<double> mag_y = 0;
inline std::atomic<double> mag_z = 0;

inline std::atomic<double> lat = 0;
inline std::atomic<double> lon = 0;
inline std::atomic<double> gps_alt = 0;
inline std::atomic<double> gps_speed = 0;


inline std::atomic<int> pyro_1_cmd = 0;
inline std::atomic<int> pyro_2_cmd = 0;



#endif