/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: launch_detect.hpp
Auth: Yubo Wang
Desc: Header file for launch detection task

*/

#pragma once

#include "arduino_freertos.h"

#include "config.hpp"
#include "sensors/IMU/imu_main.hpp"
#include "state_mgmt/state_manager.hpp"

namespace launch_detect {

    // ================================= Constants ====================================

    // =============================== Variables ======================================

    inline TaskHandle_t launch_detect_taskHandle = NULL;

    // ============================ Function Prototypes ===============================

    void launch_detect_task(void*);

} // namespace launch_detect