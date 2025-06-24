/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: gps_task.hpp
Auth: Yubo Wang
Desc: Header file for telemtry and logging thread

*/

#ifndef GPS_TASK_HPP
#define GPS_TASK_HPP

// ================================== Includes ====================================
#include "arduino_freertos.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include "config.hpp"
#include "state_mgmt/state_manager_task.hpp"

// vars
namespace sensors::gps {

    inline TaskHandle_t taskHandle = NULL;

    inline TinyGPSPlus gps_;

    void gps_task(void*);

    void gpsInit();

}

#endif // GPS_TASK_HPP
