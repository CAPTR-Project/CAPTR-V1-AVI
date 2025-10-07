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

#ifndef IMU_MAIN_H
#define IMU_MAIN_H

#include "Arduino.h"
#include "arduino_freertos.h"
#include "Adafruit_LSM6DSOX.h"
#include "captr_sensor_msgs.hpp"

#include "config.hpp"
#include "sensor_fusion/attitude_est_task.hpp"

namespace sensors::IMU_main {
    
    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline SemaphoreHandle_t i2c0_mutex_;

    inline Adafruit_LSM6DSOX imu_;
    // inline Adafruit_LSM6DS33 imu_;

    inline sensor_msgs::GyroMsg gyroData_;
    inline sensor_msgs::AccelMsg accelData_;

    inline TaskHandle_t gyro_taskHandle = NULL;

    // ============================ Function Prototypes ===============================
    void IMUInit();

    void gyroDaqISR();

    void imuDaqThread(void*);

    // void AccelDaqISR();

    // void AccelDaqThread(void*);
}

#endif // IMU_MAIN_H