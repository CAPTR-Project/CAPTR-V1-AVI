/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: main.hpp
Auth: Alex Wang, Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef TELEM_LOGGER_THREAD_HPP
#define TELEM_LOGGER_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"

// Include libs

// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void telem_logger_thread(void *params);

#endif