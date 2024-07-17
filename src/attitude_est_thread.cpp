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

std::atomic_bool last_action_was_predict = false;

void att_est_predict_thread(void*) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    long long last_time_us = micros();
    while (1) {
        ulTaskNotifyTake(1, portMAX_DELAY);
        if (mcu_state == ControllerState::LV_ON || mcu_state == ControllerState::CALIBRATING) continue;
        if (xSemaphoreTake(att_est_mutex, pdMS_TO_TICKS(5)) == pdTRUE && // TODO: change delay to match freq of gyro
            att_estimator.initialized) {
            // run UKF predict
            long long current_time_us = micros();
            att_estimator.predict((current_time_us - last_time_us) * 0.000001,
                                   Eigen::Vector3d(gyro_x, gyro_y, gyro_z));
            
            last_action_was_predict = true;
            xSemaphoreGive(att_est_mutex);
        }
    }
}

void att_est_update_thread(void*) {
    // Attitude estimation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    long long last_time_us = micros();
    while (1) {
        ulTaskNotifyTake(1, portMAX_DELAY);
        if (mcu_state == ControllerState::LV_ON || mcu_state == ControllerState::CALIBRATING) continue;
        if (last_action_was_predict) {
            if (xSemaphoreTake(att_est_mutex, pdMS_TO_TICKS(5)) == pdTRUE && // TODO: change delay to match freq of mag
            att_estimator.initialized) {
                long long current_time_us = micros();
                att_estimator.update_mag(Eigen::Vector3d(mag_x, mag_y, mag_z));
                last_action_was_predict = false;
                xSemaphoreGive(att_est_mutex);
            }
        }
    }
}