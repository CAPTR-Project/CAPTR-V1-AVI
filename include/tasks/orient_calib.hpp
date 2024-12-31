/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: orient_calib.hpp
Auth: Yubo Wang
Desc: Header file for starting orientation calibration asynchronous task. Not long-living, gets killed after calibration is done.

*/

#ifndef ORIENT_CALIB_TASK_HPP
#define ORIENT_CALIB_TASK_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"
#include "config.hpp"
#include "threads/daq_thread.hpp"
#include "quaternion.h"

namespace orient_calib_task {

// ================================= Constants ====================================

inline TaskHandle_t taskHandle = NULL;

inline std::atomic<bool> orient_calib_done = false;

// ================================= Vars ====================================
inline UnitQuaternion starting_orientation = UnitQuaternion(1, 0, 0, 0);

// ============================ Function Prototypes ==============================

void startingOrientationEstimation_task(void*);

}

#endif