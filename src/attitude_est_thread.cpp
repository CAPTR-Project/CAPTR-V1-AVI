/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: attitude_est_thread.cpp
Auth: Yubo Wang
Desc: Source file for attitude estimation thread

*/

#include "threads/attitude_est_thread.hpp"

namespace att_est_threads {

void att_est_predict_thread(void*) {

    att_estimator__.init(UnitQuaternion(1, 0, 0, 0),
                    Eigen::Vector3d(0, 0, 0),
                    Eigen::Vector3d(0, 0, 0),
                    Q_MATRIX,
                    R_MATRIX);

    att_estimator__.initialized = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    long long last_time_us = pdTICKS_TO_US(xLastWakeTime);

    sensor_msgs::GyroMsg local_gyro_data;
    ControllerState local_mcu_state;

    att_est_mutex_ = xSemaphoreCreateMutex();

    uint64_t local_current_time_us;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        local_current_time_us = current_time_us_;
        if (xSemaphoreTake(gyro_data__.ready, 0) == pdTRUE) {
            local_gyro_data = gyro_data__;
            xSemaphoreGive(gyro_data__.ready);

            // DEBUG
            // local_gyro_data.x = 0.0;
            // local_gyro_data.y = 0.0;
            // local_gyro_data.z = 0.62831853;
            
            local_mcu_state = mcu_state_.load();
            if (local_mcu_state == ControllerState::STBY || local_mcu_state == ControllerState::CALIBRATING || !att_estimator__.initialized) continue;
            if (xSemaphoreTake(att_est_mutex_, 0) == pdTRUE && // TODO: change delay to match freq of gyro
                att_estimator__.initialized) {
                
                // run UKF predict
                att_estimator__.predict_integrate((local_current_time_us - last_time_us) * 0.000001,
                                    local_gyro_data.toVector());
                // Serial.println("Local Gyro Data: " + String(local_gyro_data.x) + " " + String(local_gyro_data.y) + " " + String(local_gyro_data.z));
                // att_estimator__.predict_integrate(0.002404,
                //                     local_gyro_data.toVector());
                // Serial.println(current_time_us / 1000);
                last_action_was_predict = true;
                last_time_us = local_current_time_us;
                xSemaphoreGive(att_est_mutex_);
            }
        }
    }
}

void att_est_update_thread(void*) {
    // Attitude estimation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    long long last_time_us = pdTICKS_TO_US(xLastWakeTime);
    
    sensor_msgs::MagMsg local_mag_data;
    ControllerState local_mcu_state;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (xSemaphoreTake(mag_data__.ready, 1) == pdTRUE) {
            local_mag_data = mag_data__;
            xSemaphoreGive(mag_data__.ready);
        
            local_mcu_state = mcu_state_.load();
            if (local_mcu_state == ControllerState::STBY || local_mcu_state == ControllerState::CALIBRATING) continue;
            if (last_action_was_predict) {
                if (xSemaphoreTake(att_est_mutex_, pdMS_TO_TICKS(23)) == pdTRUE && // TODO: change delay to match freq of mag
                att_estimator__.initialized) {
                    // Get mag data
                    long long current_time_us = pdTICKS_TO_US(xTaskGetTickCount());
                    att_estimator__.update_mag(local_mag_data.toVector());
                    last_action_was_predict = false;
                    xSemaphoreGive(att_est_mutex_);
                }
            }
        }
    }
}

} // namespace att_est_threads