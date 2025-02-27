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

namespace daq_threads {

SemaphoreHandle_t i2c0_mutex;

void daq_start() {
    i2c0_mutex = xSemaphoreCreateMutex();

    xTaskCreate(gyro_daq_thread, "Gyro DAQ", 4000, nullptr, 8, &gyro_taskHandle);
    xTaskCreate(accel_daq_thread, "Accel DAQ", 4000, nullptr, 8, &accel_taskHandle);
    xTaskCreate(mag_daq_thread, "Mag DAQ", 4000, nullptr, 8, &mag_taskHandle);
    xTaskCreate(baro_daq_thread, "Baro DAQ", 4000, nullptr, 8, &baro_taskHandle);

    vPortEnterCritical();

    if (!sensors_lib::initIMU_LSM6DSO32(&imu__, LSM6DS_I2CADDR_DEFAULT, &Wire, IMU_DATARATE, accelISR, ACCEL_INT_PIN, gyroISR, GYRO_INT_PIN)) {
        error_state_.store(ErrorState::IMU);
    }
    // if (!sensors_lib::initMag(&mag__, 0x1E, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN)) {
    //     error_state_.store(ErrorState::MAG);
    // }
    if (!sensors_lib::initBMP(&bmp__, BMP3XX_DEFAULT_ADDRESS, &Wire1, BARO_DATARATE, baroISR, BARO_INT_PIN)) {
        error_state_.store(ErrorState::BARO);
    }


    vPortExitCritical();


}

void gyro_daq_thread(void*) {
    gyro_data__ = sensor_msgs::GyroMsg();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Serial.println("Gyro DAQ");
        if (xSemaphoreTake(gyro_data__.ready, 1) == pdTRUE && xSemaphoreTake(i2c0_mutex, 1) == pdTRUE) {
            if (!imu__.readGyroscope(gyro_data__.x, gyro_data__.y, gyro_data__.z)) {
                error_state_ = ErrorState::GYRO;
            }
            xSemaphoreGive(gyro_data__.ready);
            xSemaphoreGive(i2c0_mutex);

            if ( att_est_threads::predictTaskHandle_ != NULL ) {
                att_est_threads::current_time_us_ = pdTICKS_TO_US(xTaskGetTickCount());
                xTaskNotifyGive(att_est_threads::predictTaskHandle_);
            }
            if (gyro_calib_task::taskHandle != NULL) {
                xTaskNotifyGive(gyro_calib_task::taskHandle);
            }
        }
    }
}

void accel_daq_thread(void*) {
    accel_data__ = sensor_msgs::AccelMsg();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Serial.println("Accel DAQ");
        if (xSemaphoreTake(accel_data__.ready, 1) == pdTRUE && xSemaphoreTake(i2c0_mutex, 4) == pdTRUE) {
            if (!imu__.readAcceleration(accel_data__.x, accel_data__.y, accel_data__.z)) {
                error_state_ = ErrorState::ACCEL;
            }
            xSemaphoreGive(accel_data__.ready);
            xSemaphoreGive(i2c0_mutex);
            if (orient_calib_task::taskHandle != NULL) {
                xTaskNotifyGive(orient_calib_task::taskHandle);
            }
        }
    }
}

void mag_daq_thread(void*) {
    mag_data__ = sensor_msgs::MagMsg();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(mag_data__.ready, 1) == pdTRUE) {
            mag__.read();
            // sensor_msgs::MagMsg new_mag_data = sensor_msgs::MagMsg();
            // new_mag_data.x = mag__.x_gauss + MAG_X_OFFSET;
            // new_mag_data.y = mag__.y_gauss + MAG_Y_OFFSET;
            // new_mag_data.z = mag__.z_gauss + MAG_Z_OFFSET;
            mag_data__.x = mag__.x_gauss + MAG_X_OFFSET;
            mag_data__.y = mag__.y_gauss + MAG_Y_OFFSET;
            mag_data__.z = mag__.z_gauss + MAG_Z_OFFSET;
            xSemaphoreGive(mag_data__.ready);

            if (att_est_threads::updateTaskHandle_ != NULL ) {
                xTaskNotifyGive(att_est_threads::updateTaskHandle_);
            }

            if (mag_calib_task::taskHandle != NULL) {
                xTaskNotifyGive(mag_calib_task::taskHandle);
            }
        }
    }
}

void baro_daq_thread(void*) {
    baro_data__ = sensor_msgs::BaroMsg();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(baro_data__.ready, 1)) {
            baro_data__.alt_msl = bmp__.readAltitude(BARO_PRESSURE_ASL * 0.01);

            baro_data__.alt_agl = baro_data__.alt_msl - ground_altitude_offset_msl__;

            baro_data__.pressure = bmp__.pressure;
            xSemaphoreGive(baro_data__.ready);
        }
    }
}

} // namespace daq_thread