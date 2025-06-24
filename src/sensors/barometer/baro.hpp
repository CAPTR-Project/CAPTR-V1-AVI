/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: baro.hpp
Auth: Yubo Wang
Desc: Header file for barometer data acquisition

*/

#include "Arduino.h"
#include "Adafruit_BMP3XX.h"
#include "captr_sensor_msgs.hpp"

#include "config.hpp"

#ifndef BARO_HPP
#define BARO_HPP

namespace sensors::baro {

    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline Adafruit_BMP3XX bmp_ = Adafruit_BMP3XX();
    inline sensor_msgs::BaroMsg baroData_;

    inline TaskHandle_t baro_taskHandle = NULL;

    inline double ground_altitude_offset_msl_ = 0;

    // ============================ Function Prototypes ===============================
    void baroInit();

    void baroDaqISR(void);

    void baroDaqThread(void*);

} // namespace barometer

#endif // BARO_HPP