/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: daq_thread.cpp
Auth: Yubo Wang
Desc: Source file for telemetry and logging thread

*/

#include "threads/daq_thread.hpp"

namespace daq_thread {

void daq_thread(void*) {

    // init sensors
    if (!sensors_lib::initIMU(&imu__, LSM6DS_I2CADDR_DEFAULT, &Wire, IMU_DATARATE, imuISR, ACCEL_INT_PIN, gyroISR, GYRO_INT_PIN)) {
        error_state_.store(ErrorState::IMU);
    }
    if (!sensors_lib::initMag(&mag__, 0x1E, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN, true)) {
        error_state_.store(ErrorState::MAG);
    }
    if (!sensors_lib::initBMP(&bmp__, BMP3_ADDR_I2C_SEC, &Wire1, baroISR, BARO_INT_PIN)) {
        error_state_.store(ErrorState::BARO);
    }

     while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("Data received");
        
        // Read data
        if (accel_data_ready_ && xSemaphoreTake(accel_data__.ready, 1)) {
            accel_data__ = temp_accel_data_;
            accel_data_ready_ = false;
        }
        if (gyro_data_ready_) {
            if (xSemaphoreTake(gyro_data__.ready, 1) == pdTRUE) {
                gyro_data__ = temp_gyro_data_;
                gyro_data_ready_ = false;
                xSemaphoreGive(gyro_data__.ready);

                configASSERT( att_est_threads::predictTaskHandle_ != NULL );
                xTaskNotifyGive(att_est_threads::predictTaskHandle_);
                if (gyro_calib_task::taskHandle != NULL) {
                    xTaskNotifyGive(gyro_calib_task::taskHandle);
                }
            }
        }
        if (mag_data_ready_ && xSemaphoreTake(mag_data__.ready, 1)) {
            if (xSemaphoreTake(mag_data__.ready, 1) == pdTRUE) {
                mag_data__ = temp_mag_data_;
                mag_data_ready_ = false;
                xSemaphoreGive(mag_data__.ready);

                configASSERT( att_est_threads::updateTaskHandle_ != NULL );
                xTaskNotifyGive(att_est_threads::updateTaskHandle_);

                if (mag_calib_task::taskHandle != NULL) {
                    xTaskNotifyGive(mag_calib_task::taskHandle);
                }
            }
        }
        if (baro_data_ready_ && xSemaphoreTake(baro_data__.ready, 1)) {
            baro_data__ = temp_baro_data_;
            baro_data_ready_ = false;
        }
    }
}

} // namespace daq_thread