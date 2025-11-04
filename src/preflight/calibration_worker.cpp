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
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
            xLastWakeTime = xTaskGetTickCount();
        }
        gyroBiasX = gyroX / cnt;
        gyroBiasY = gyroY / cnt;
        gyroBiasZ = gyroZ / cnt;

        accelX = accelX / cnt;
        accelY = accelY / cnt;
        accelZ = accelZ / cnt;

        magX = magX / cnt;
        magY = magY / cnt;
        magZ = magZ / cnt;

        // Calculate starting orientation (roll/pitch only) from gravity
        Eigen::Vector3d g_b = Eigen::Vector3d(accelX, accelY, accelZ).normalized();

        // Choose inertial gravity direction
        Eigen::Vector3d g_i(1, 0, 0);

        // Robust quaternion: rotate body -> inertial so that g_b maps to g_i
        Eigen::Vector3d v = g_b.cross(g_i);
        double c = std::clamp(g_b.dot(g_i), -1.0, 1.0);
        double s2 = (1.0 + c) * 2.0;

        Eigen::Quaterniond q_BI;
        if (s2 < 1e-8) {
            // g_b and g_i are opposite; choose any orthogonal axis
            Eigen::Vector3d axis = g_b.unitOrthogonal();
            q_BI = Eigen::AngleAxisd(M_PI, axis);
        } else {
            double s = std::sqrt(s2);
            q_BI.w() = 0.5 * s;
            q_BI.x() = v.x() / s;
            q_BI.y() = v.y() / s;
            q_BI.z() = v.z() / s;
        }
        q_BI.normalize();

        // Save for later if you need it in your UnitQuaternion type
        startingOrientation = UnitQuaternion(q_BI.w(), q_BI.x(), q_BI.y(), q_BI.z());

        // Tilt-compensate magnetometer: inertial mag vector is obtained by rotating the body mag vector
        Eigen::Vector3d mag_b(magX, magY, magZ);
        mag_vec_ = startingOrientation.vector_rotation_by_quaternion(mag_b);

        // Serial.printf("Mag vector in inertial frame: %.2f, %.2f, %.2f\n",
        //       mag_vec_(0), mag_vec_(1), mag_vec_(2));
       
        calibration_done = true;

        Serial.println("Calibration done");

        state_manager::requestState(MCUState::LAUNCH_DETECT);

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