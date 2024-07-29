/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: gyro_calib.hpp
Auth: Yubo Wang
Desc: Header file for gyroscope calibration asynchronous task. Not long-living, gets killed after calibration is done.

*/

#ifndef GYRO_CALIB_TASK_HPP
#define GYRO_CALIB_TASK_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"


// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void gyro_calibration(void*);

#endif