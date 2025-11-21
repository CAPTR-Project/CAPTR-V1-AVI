/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: sd_logging_task.hpp
Auth: Yubo Wang
Desc: Header file for SD logging thread

*/


#ifndef SD_LOGGING_TASK_HPP
#define SD_LOGGING_TASK_HPP

#include "arduino_freertos.h"
#include <SPI.h>
#include <SD.h>

#include "config.hpp"
#include "sensors/sensors.hpp"
#include "state_mgmt/state_manager_task.hpp"
#include "sensor_fusion/attitude_est_task.hpp"

namespace sd_logging {
// ============================== Constants/Vars ==================================
inline TaskHandle_t taskHandle = NULL;
inline File logfile;
inline File itertrackfile;

// ============================ Structs/Enums/Unions ==============================


// ============================ Function Prototypes ==============================

void sd_logging_task(void*);

}

#endif