/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: state_mgmt_thread.hpp
Auth: Yubo Wang
Desc: Header file for control thread

*/

#ifndef STATE_THREAD_HPP
#define STATE_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "config.hpp"
#include "globals.hpp"

#include "daq_ISRs.hpp"

// CAPTR libs
#include "CAPTR_PIN_DRIVER.hpp"
#include "sensors_setup.hpp"
#include "attitude_estimator.hpp"
#include "quaternion.h"

// threads
#include "threads/attitude_est_thread.hpp"
#include "threads/control_thread.hpp"
#include "threads/daq_thread.hpp"
#include "threads/datalogger_thread.hpp"

// tasks
#include "tasks/gyro_calib.hpp"
#include "tasks/mag_calib.hpp"

namespace state_mgmt_thread {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;
inline uint32_t last_state_change_ms = 0;

// ============================ Function Prototypes ==============================

void state_mgmt_thread(void*);

}

#endif