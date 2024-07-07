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

inline long long msElapsed = 0;

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
SemaphoreHandle_t att_est_mutex;


#endif