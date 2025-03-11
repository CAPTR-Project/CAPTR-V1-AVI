/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: imu.hpp
Auth: Yubo Wang
Desc: Header file for IMU data acquisition

*/

#include "captr_sensor_msgs.hpp"

#pragma once

namespace sensors::IMU_main {
    
    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline sensor_msgs::GyroMsg gyroData_;
    inline sensor_msgs::AccelMsg accelData_;

    // ============================ Function Prototypes ===============================
    void IMUinit(void*);

    void GyroDaqISR();

    void GyroDaqThread(void*);

    void AccelDaqISR();

    void AccelDaqThread(void*);
}