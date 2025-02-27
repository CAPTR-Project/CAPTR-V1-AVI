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

namespace mag {

    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline Adafruit_LIS3MDL mag_sensor_ = Adafruit_LIS3MDL();
    inline sensor_msgs::MagMsg mag_data_;

    // ============================ Function Prototypes ===============================
    void mag_init(void*);

    void mag_daq_ISR(void);

    void mag_daq_thread(void*);

} // namespace mag