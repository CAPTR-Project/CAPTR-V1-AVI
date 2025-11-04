/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: calibration_worker.hpp
Auth: Yubo Wang
Desc: Header file for Attitude Estimation Thread

*/

#ifndef CALIBRATION_WORKER_HPP
#define CALIBRATION_WORKER_HPP

#include "arduino_freertos.h"
#include "ArduinoEigenDense.h"
#include "quaternion.h"

#include "config.hpp"
#include "state_mgmt/state_manager_task.hpp"
#include "attitude_estimator.hpp"
#include "captr_sensor_msgs.hpp"
#include "sensors/IMU/imu_main.hpp"
#include "sensors/mag/mag.hpp"
#include "sensors/barometer/baro.hpp"

namespace calibration_worker {

    // ================================= Constants ====================================

    // =============================== Variables ======================================

    inline TaskHandle_t calibrate_taskHandle = NULL;

    inline bool calibration_done = false;

    inline bool stop_flag = false;

    inline double gyroBiasX, gyroBiasY, gyroBiasZ;

    inline Eigen::Vector3d mag_vec_;

    inline UnitQuaternion startingOrientation;

    // ============================ Function Prototypes ===============================

    void calibrate_task(void*);
    void stop_calibration();

} // namespace calibration_worker

#endif // CALIBRATION_WORKER_HPP