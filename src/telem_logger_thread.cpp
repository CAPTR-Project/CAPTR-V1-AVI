/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: telem_logger_thread.cpp
Auth: Yubo Wang
Desc: Source file for telemetry and logging thread

*/

#include "threads/telem_logger_thread.hpp"

// define a few global variables and assume they're gonna be updated properly

namespace telem_logger_thread {

void telem_logger_thread(void*) {
    // Telemetry and logging
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;

    sensor_msgs::AccelMsg local_accel_data;
    sensor_msgs::GyroMsg local_gyro_data;
    sensor_msgs::MagMsg local_mag_data;
    sensor_msgs::BaroMsg local_baro_data;
    ControllerState local_mcu_state;

    while (1) {
        // Telemetry and logging loop
        if (xSemaphoreTake(reading_flag_, 0) == pdTRUE) {
            local_accel_data = accel_data_;
            local_gyro_data = gyro_data_;
            local_mag_data = mag_data_;
            local_baro_data = baro_data_;
            xSemaphoreGive(reading_flag_);
        }
        if (xSemaphoreTake(att_estimator.ready, 0) == pdTRUE) {
            attitude_ = att_estimator.newest_attitude_quat;
            xSemaphoreGive(att_estimator.ready);
        }
        local_mcu_state = mcu_state_.load();

        // Log data

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        if (!xWasDelayed) {
            // Log error
        }
    }
}

void telem_logger_DAQ_thread(void*) {
     while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (xSemaphoreTake(reading_flag_, 0) == pdTRUE) {
            // Read data
            if (accel_data_ready_) {
                accel_data_ = accel_data_;
                accel_data_ready_ = false;
            }
            if (gyro_data_ready_) {
                gyro_data_ = gyro_data_;
                gyro_data_ready_ = false;
            }
            if (mag_data_ready_) {
                mag_data_ = mag_data_;
                mag_data_ready_ = false;
            }
            if (baro_data_ready_) {
                baro_data_ = baro_data_;
                baro_data_ready_ = false;
            }

            xSemaphoreGive(reading_flag_);
        }
     }
}

} // namespace telem_logger_thread