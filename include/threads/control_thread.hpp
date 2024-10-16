/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: control_thread.hpp
Auth: Yubo Wang
Desc: Header file for control thread

*/

#ifndef CONTROL_THREAD_HPP
#define CONTROL_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "config.hpp"
#include "globals.hpp"
#include "attitude_estimator.hpp"

namespace controls_thread {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif