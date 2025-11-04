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

#include "attitude_est_task.hpp"

namespace att_est_tasks {

void att_est_predict_thread(void*) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    long long last_time_us = pdTICKS_TO_US(xLastWakeTime);

    sensor_msgs::GyroMsg local_gyro_data;

    att_est_mutex_ = xSemaphoreCreateMutex();

    uint64_t local_current_time_us;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        local_current_time_us = current_time_us_;
        local_gyro_data = sensors::IMU_main::gyroData_;
        double dt = (local_current_time_us - last_time_us) * 0.000001;
        // DEBUG
        // local_gyro_data.x = 0.0;
        // local_gyro_data.y = 0.0;
        // local_gyro_data.z = 0.62831853;
        
        if (xSemaphoreTake(att_est_mutex_, 0) == pdTRUE && // TODO: change delay to match freq of gyro
            att_estimator_.initialized) {
            
            // run UKF predict
            att_estimator_.predict(dt,
                                local_gyro_data.toVector());
            // att_estimator_.predict_integrate(0.002404,
            //                     local_gyro_data.toVector());
            // Serial.println(current_time_us / 1000);
            xSemaphoreGive(att_est_mutex_);
        }
        last_time_us = local_current_time_us;
        // if (dt > 1/416.0+0.001) {
        //     Serial.println("Warning: Attitude est prediction step too large: rate=" + String(1.0/dt) + " Hz");
        // }
    }
}

void att_est_update_thread(void*) {
    // Attitude estimation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    long long last_time_us = pdTICKS_TO_US(xLastWakeTime);
    
    sensor_msgs::MagMsg localMagData;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        localMagData = sensors::mag::magData_;
    
        if (xSemaphoreTake(att_est_mutex_, pdMS_TO_TICKS(100)) == pdTRUE && // TODO: change delay to match freq of mag
        att_estimator_.initialized) {
            // Get mag data
            long long current_time_us = pdTICKS_TO_US(xTaskGetTickCount());
            att_estimator_.update_mag(localMagData.toVector());
            last_action_was_predict = false;
            xSemaphoreGive(att_est_mutex_);
        }
    }
}

} // namespace att_est_tasks