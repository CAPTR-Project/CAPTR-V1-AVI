/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: daq_threads.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef TELEM_LOGGER_THREAD_HPP
#define TELEM_LOGGER_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"
#include "quaternion.h"
#include "attitude_est_thread.hpp"
#include "tasks/gyro_calib.hpp"
#include "tasks/mag_calib.hpp"
#include "sensors_setup.hpp"
#include "daq_ISRs.hpp"

namespace daq_threads {

// Include libs

// ================================= Vars ====================================

// inline TaskHandle_t taskHandle = NULL;
inline TaskHandle_t gyro_taskHandle = NULL;
inline TaskHandle_t mag_taskHandle = NULL;
inline TaskHandle_t baro_taskHandle = NULL;
inline TaskHandle_t accel_taskHandle = NULL;

inline float gyro_bias_x_ = 0;
inline float gyro_bias_y_ = 0;
inline float gyro_bias_z_ = 0;

// Include constants/variables

// ============================ Function Prototypes ==============================

void daq_start();
void gyro_daq_thread(void*);
void accel_daq_thread(void*);
void magDaqThread(void*);
void baro_daq_thread(void*);

} // namespace daq_threads

#endif