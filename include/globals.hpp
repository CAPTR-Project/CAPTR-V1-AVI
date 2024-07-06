#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "rtos_includes.hpp"

inline long long msElapsed = 0;

// STATE MACHINE
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

#endif