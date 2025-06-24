/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: calibration_worker.cpp
Auth: Yubo Wang
Desc: Source file for Calibration routine

*/

#include "calibration_worker.hpp"

namespace calibration_worker {
    
    void calibrate_task(void*) {
        if (state_manager::currentState != MCUState::CALIBRATING) {
            vTaskDelete(NULL);
            return;
        }

        // Initialize variables
        calibration_done = false;

        long cnt = 0;
        sensor_msgs::GyroMsg localGyroData;
        sensor_msgs::AccelMsg localAccelData;
        sensor_msgs::MagMsg localMagData;


        float gyroX = 0;
        float gyroY = 0;
        float gyroZ = 0;

        float accelX = 0;
        float accelY = 0;
        float accelZ = 0;

        float magX = 0;
        float magY = 0;
        float magZ = 0;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;
        TickType_t start_time = xTaskGetTickCount();

        stop_flag = false;
        while (xLastWakeTime < start_time + pdMS_TO_TICKS(GYRO_CALIBRATION_TIME) && !stop_flag) {
            // Read gyro data
            localGyroData = sensors::IMU_main::gyroData_;
            localAccelData = sensors::IMU_main::accelData_;
            localMagData = sensors::mag::magData_;
        
            gyroX += localGyroData.x;
            gyroY += localGyroData.y;
            gyroZ += localGyroData.z;

            accelX += localAccelData.x;
            accelY += localAccelData.y;
            accelZ += localAccelData.z;

            magX += localMagData.x;
            magY += localMagData.y;
            magZ += localMagData.z;

            cnt++;
            xLastWakeTime = xTaskGetTickCount();
        }
        gyroBiasX = gyroX / cnt;
        gyroBiasY = gyroY / cnt;
        gyroBiasZ = gyroZ / cnt;

        accelX = accelX / cnt;
        accelY = accelY / cnt;
        accelZ = accelZ / cnt;

        magStartX = magX / cnt;
        magStartY = magY / cnt;
        magStartZ = magZ / cnt;

        // Calculate starting orientation
        Eigen::Vector3d normalized = Eigen::Vector3d(accelX, accelY, accelZ).normalized();
        Eigen::Vector3d up = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d axis = up.cross(normalized);
        double angle = acos(up.dot(normalized) / (up.norm() * normalized.norm()));
        startingOrientation = UnitQuaternion::from_rotVec(angle * axis(0), angle * axis(1), angle * axis(2)).conjugate();

        calibration_done = true;

        Serial.println("Gyro calibration done");

        state_manager::requestState(MCUState::STBY);

        vTaskDelete(NULL);
        return;
    }
    void stop_calibration() {
        if (calibration_done) {
            return;
        }
        stop_flag = true;
        // Wait for the calibration task to finish
        while (calibration_done == false) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        calibration_done = false;
    }

} // namespace calibration_worker