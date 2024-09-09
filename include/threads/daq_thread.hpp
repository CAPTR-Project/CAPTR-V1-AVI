/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: daq_thread.hpp
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

namespace daq_thread {

// Include libs

// ================================= Vars ====================================

inline TaskHandle_t taskHandle = NULL;

inline sensor_msgs::AccelMsg temp_accel_data_;
inline std::atomic_bool accel_data_ready_ = false;
inline sensor_msgs::GyroMsg temp_gyro_data_;
inline std::atomic_bool gyro_data_ready_ = false;
inline sensor_msgs::MagMsg temp_mag_data_;
inline std::atomic_bool mag_data_ready_ = false;
inline sensor_msgs::BaroMsg temp_baro_data_;
inline std::atomic_bool baro_data_ready_ = false;

// Include constants/variables

// ============================ Function Prototypes ==============================

void daq_thread(void*);
void telem_logger_DAQ_thread(void*);

} // namespace daq_thread

#endif