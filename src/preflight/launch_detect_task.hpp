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

#ifndef LAUNCH_DETECT_TASK_HPP
#define LAUNCH_DETECT_TASK_HPP

#include "arduino_freertos.h"

#include "config.hpp"
#include "sensors/IMU/imu_main.hpp"
#include "state_mgmt/state_manager_task.hpp"

namespace launch_detect {

    // ================================= Constants ====================================

    // =============================== Variables ======================================

    inline TaskHandle_t taskHandle = NULL;

    // ============================ Function Prototypes ===============================

    void launch_detect_task(void*);

} // namespace launch_detect

#endif // LAUNCH_DETECT_TASK_HPP