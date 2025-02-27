/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: gps_thread.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef GPS_THREAD_HPP
#define GPS_THREAD_HPP

// ================================== Includes ====================================
#include "rtos_includes.hpp"
#include "globals.hpp"
#include "config.hpp"
#include "sensors_setup.hpp"

// vars
namespace gps_thread {

    inline TaskHandle_t taskHandle = NULL;

}

#endif // GPS_THREAD_HPP
