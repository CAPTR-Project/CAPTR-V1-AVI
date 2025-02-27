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

#include <Arduino.h>
#include <atomic>

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