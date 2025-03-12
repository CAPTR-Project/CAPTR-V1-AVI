/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: mag.hpp
Auth: Yubo Wang
Desc: Header file for magnetometer data acquisition

*/

#pragma once

#include <Arduino.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include "captr_sensor_msgs.hpp"

#include "config.hpp"
#include "sensors/IMU/imu_main.hpp"

namespace sensors::mag {

    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline Adafruit_LIS3MDL mag_ = Adafruit_LIS3MDL();
    inline sensor_msgs::MagMsg magData_;

    inline TaskHandle_t mag_taskHandle = NULL;

    // ============================ Function Prototypes ===============================
    void magInit(void*);

    void magDaqISR(void);

    void magDaqThread(void*);

} // namespace mag