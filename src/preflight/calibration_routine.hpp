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

#include "config.hpp"
#include "attitude_estimator.hpp"
#include "captr_sensor_msgs.hpp"
#include "sensors/IMU/imu.hpp"
#include "sensors/mag/mag.hpp"
#include "sensors/baro/baro.hpp"

namespace calibration_routine {

    void calibrate_task(void*);

} // namespace calibration_routine