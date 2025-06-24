/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: imu_.hpp
Auth: Yubo Wang
Desc: Source file for imu_ data acquisition

*/

#include "imu_main.hpp"

namespace sensors::IMU_main {

    void IMUInit() {
        
        if (!imu_.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire)) {
            Serial.println("ERROR: Failed to find LSM6DS chip");
        }

        Serial.println("LSM6DS Found!");

        imu_.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
        imu_.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        imu_.setAccelDataRate(ACCEL_DATARATE);
        imu_.setGyroDataRate(GYRO_DATARATE);

        imu_.getGyroRange();
        imu_.getAccelRange();

        vPortEnterCritical();

        imu_.configInt1(false, true, false); // enable interrupt on gyroscope data ready
        imu_.configInt2(false, false, true); // enable interrupt on accelerometer data ready
        imu_.configIntOutputs(false, false); // set to active high and push-pull

        // attachInterrupt(digitalPinToInterrupt(accel_isr_pin), imu__isr, arduino::RISING);
        attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), gyroDaqISR, arduino::RISING);

        // (*imu__isr)(); // read once to clear interrupt
        // (*gyro_isr)(); // read once to clear interrupt
        float x, y, z;
        imu_.readAcceleration(x, y, z);
        imu_.readGyroscope(x, y, z);

        vPortExitCritical();

        xTaskCreate(imuDaqThread, "Gyro DAQ", 4000, nullptr, 8, &gyro_taskHandle);
        Serial.println("Gyro DAQ thread created");
    }

    void gyroDaqISR() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Send notification to control task on INDEX 0. 
        configASSERT( gyro_taskHandle != NULL );
        vTaskNotifyGiveFromISR( gyro_taskHandle, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    void imuDaqThread(void*) {

        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // Serial.println("Gyro DAQ");
            if (xSemaphoreTake(i2c0_mutex_, 1) == pdTRUE) {
                imu_.read();
                // if (!imu_.readGyroscope(gyroData_.x, gyroData_.y, gyroData_.z)) {
                //     error_state_ = ErrorState::GYRO;
                // }
                gyroData_.x = imu_.gyroX; gyroData_.y = imu_.gyroY; gyroData_.z = imu_.gyroZ;

                accelData_.x = imu_.accX; accelData_.y = imu_.accY; accelData_.z = imu_.accZ;

                xSemaphoreGive(gyroData_.ready);
                xSemaphoreGive(i2c0_mutex_);

                if ( att_est_tasks::predictTaskHandle_ != NULL ) {
                    att_est_tasks::current_time_us_ = pdTICKS_TO_US(xTaskGetTickCount());
                    xTaskNotifyGive(att_est_tasks::predictTaskHandle_);
                }
            }
        }
    }

    // void AccelDaqISR() {

    // }

    // void AccelDaqThread(void*) {

    // }

}