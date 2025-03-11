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

#include "Arduino.h"
#include "arduino_freertos.h"
#include "Adafruit_LSM6DSOX.h"
#include "captr_sensor_msgs.hpp"

#include "config.hpp"
#include "sensor_fusion/attitude_est_thread.hpp"

#pragma once

namespace sensors::IMU_main {
    
    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline SemaphoreHandle_t i2c0_mutex_;

    Adafruit_LSM6DSOX imu_;

    inline sensor_msgs::GyroMsg gyroData_;
    inline sensor_msgs::AccelMsg accelData_;

    TaskHandle_t gyro_taskHandle = NULL;

    // ============================ Function Prototypes ===============================
    void IMUInit();

    void GyroDaqISR();

    void GyroDaqThread(void*);

    // void AccelDaqISR();

    // void AccelDaqThread(void*);
}