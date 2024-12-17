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

    baro_data__ = sensor_msgs::BaroMsg();
    accel_data__ = sensor_msgs::AccelMsg();
    gyro_data__ = sensor_msgs::GyroMsg();
    mag_data__ = sensor_msgs::MagMsg();

    // init sensors
    if (!sensors_lib::initIMU(&imu__, LSM6DS_I2CADDR_DEFAULT, &Wire, IMU_DATARATE, imuISR, ACCEL_INT_PIN, gyroISR, GYRO_INT_PIN)) {
        error_state_.store(ErrorState::IMU);
    }
    if (!sensors_lib::initMag(&mag__, 0x1E, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN)) {
        error_state_.store(ErrorState::MAG);
    }
    // mag_data__.ready = xSemaphoreCreateMutex();

    if (!sensors_lib::initBMP(&bmp__, BMP3XX_DEFAULT_ADDRESS, &Wire1, BARO_DATARATE, baroISR, BARO_INT_PIN)) {
        error_state_.store(ErrorState::BARO);
    }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Serial.println("DAQ thread");
        
        // Read data
        if (accel_data_ready_ && xSemaphoreTake(accel_data__.ready, 1)) {
            accel_data__.x = temp_accel_data_.x;
            accel_data__.y = temp_accel_data_.y;
            accel_data__.z = temp_accel_data_.z;
            accel_data_ready_ = false;
            xSemaphoreGive(accel_data__.ready);
        }
        if (gyro_data_ready_) {
            if (xSemaphoreTake(gyro_data__.ready, 1) == pdTRUE) {
                // gyro_data__.x = temp_gyro_data_.x / 180.0 * M_PI;
                // gyro_data__.y = temp_gyro_data_.y / 180.0 * M_PI;
                // gyro_data__.z = temp_gyro_data_.z / 180.0 * M_PI;
                gyro_data__.x = temp_gyro_data_.x;
                gyro_data__.y = temp_gyro_data_.y;
                gyro_data__.z = temp_gyro_data_.z;
                gyro_data_ready_ = false;
                xSemaphoreGive(gyro_data__.ready);

                if ( att_est_threads::predictTaskHandle_ != NULL ) {
                    xTaskNotifyGive(att_est_threads::predictTaskHandle_);
                }
                if (gyro_calib_task::taskHandle != NULL) {
                    xTaskNotifyGive(gyro_calib_task::taskHandle);
                }
            }
        }
        if (mag_data_ready_) {
            
            if (xSemaphoreTake(mag_data__.ready, 1) == pdTRUE) {
                mag_data__.x = temp_mag_data_.x;
                mag_data__.y = temp_mag_data_.y;
                mag_data__.z = temp_mag_data_.z;
                mag_data_ready_ = false;
                xSemaphoreGive(mag_data__.ready);

                if (att_est_threads::updateTaskHandle_ != NULL ) {
                    xTaskNotifyGive(att_est_threads::updateTaskHandle_);
                }

                if (mag_calib_task::taskHandle != NULL) {
                    xTaskNotifyGive(mag_calib_task::taskHandle);
                }
            }
        }
        if (baro_data_ready_ && xSemaphoreTake(baro_data__.ready, 1)) {
            baro_data__.alt_agl = temp_baro_data_.alt_agl;
            baro_data__.alt_msl = temp_baro_data_.alt_msl;
            baro_data__.pressure = temp_baro_data_.pressure;
            baro_data_ready_ = false;
            xSemaphoreGive(baro_data__.ready);
        }
    }
}

} // namespace daq_thread