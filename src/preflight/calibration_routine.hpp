/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: calibration_routine.hpp
Auth: Yubo Wang
Desc: Header file for Attitude Estimation Thread

*/

#pragma once

#include "arduino_freertos.h"
#include "ArduinoEigenDense.h"
#include "quaternion.h"

#include "config.hpp"
#include "state_mgmt/state_manager.hpp"
#include "attitude_estimator.hpp"
#include "captr_sensor_msgs.hpp"
#include "sensors/IMU/imu_main.hpp"
#include "sensors/mag/mag.hpp"
#include "sensors/barometer/baro.hpp"

namespace calibration_routine {

    // ================================= Constants ====================================

    // =============================== Variables ======================================

    inline TaskHandle_t calibrate_taskHandle = NULL;

    inline bool calibration_done = false;

    inline bool stop_flag = false;

    inline float gyroBiasX, gyroBiasY, gyroBiasZ;

    inline float magStartX, magStartY, magStartZ;

    inline UnitQuaternion startingOrientation;

    // ============================ Function Prototypes ===============================

    void calibrate_task(void*);
    void stop_calibration();

} // namespace calibration_routine