/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: main.hpp
Auth: Alex Wang, Yubo Wang
Desc: Header file for control thread

*/

#ifndef CONTROL_THREAD_HPP
#define CONTROL_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"


// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void control_thread(void *params);

#endif