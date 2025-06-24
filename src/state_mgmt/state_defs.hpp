/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: state_defs.hpp
Auth: Alex Wang, Yubo Wang
Desc: States that the system can be in
*/

#pragma once

#ifndef STATE_DEFS_HPP
#define STATE_DEFS_HPP

#include <Arduino.h>
#include <atomic>

enum class MCUState
{
    STBY,
    SERVOTEST,
    CALIBRATING,
    LAUNCH_DETECT,
    POWERED_ASCENT,
    COAST,
    RECOVERY,
    // POWERED_DESCENT,
    LANDED,
    ERROR,
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

#endif // STATE_DEFS_HPP