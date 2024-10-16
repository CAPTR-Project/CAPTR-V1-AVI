/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: gyro_calib.hpp
Auth: Yubo Wang
Desc: Header file for magnetometer "down vector" asynchronous task. Not long-living, gets killed after calibration is done.

*/

#ifndef MAG_CALIB_TASK_HPP
#define MAG_CALIB_TASK_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"
#include "config.hpp"
#include "captr_sensor_msgs.hpp"

namespace mag_calib_task {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;

inline std::atomic_bool mag_calib_done = false;
// ============================ Function Prototypes ==============================

void magVectorEstimation_task(void*);

}

#endif