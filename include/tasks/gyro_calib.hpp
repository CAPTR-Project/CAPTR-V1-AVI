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
#include "config.hpp"
#include "threads/daq_thread.hpp"

namespace gyro_calib_task {

// ================================= Constants ====================================

inline TaskHandle_t taskHandle = NULL;

inline std::atomic<bool> gyro_calib_done = false;

// ============================ Function Prototypes ==============================

void gyroBiasEstimation_task(void*);

}

#endif