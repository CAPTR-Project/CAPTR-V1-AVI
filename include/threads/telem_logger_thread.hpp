/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: telem_logger_thread.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef TELEM_LOGGER_THREAD_HPP
#define TELEM_LOGGER_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"
#include "quaternion.h"

namespace telem_logger_thread {

// Include libs

// ================================= Vars ====================================
inline SemaphoreHandle_t reading_flag_ = NULL;

inline TaskHandle_t taskHandle = NULL;
inline TaskHandle_t daqTaskHandle = NULL;

inline sensor_msgs::AccelMsg accel_data_;
inline std::atomic_bool accel_data_ready_ = false;
inline sensor_msgs::GyroMsg gyro_data_;
inline std::atomic_bool gyro_data_ready_ = false;
inline sensor_msgs::MagMsg mag_data_;
inline std::atomic_bool mag_data_ready_ = false;
inline sensor_msgs::BaroMsg baro_data_;
inline std::atomic_bool baro_data_ready_ = false;
inline UnitQuaternion attitude_;

// ============================ Function Prototypes ==============================

void telem_logger_thread(void*);
void telem_logger_DAQ_thread(void*);

} // namespace telem_logger_thread

#endif