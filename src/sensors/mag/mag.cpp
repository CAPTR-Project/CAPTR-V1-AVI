/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: mag.hpp
Auth: Yubo Wang
Desc: Source file for magnetometer data acquisition

*/

#include "mag.hpp"

namespace sensors::mag {

    void magInit() {
        if (!mag_.begin_I2C(0x1E, &Wire)) {
            Serial.println("ERROR: Failed to find LIS3MDL chip");
        }

        Serial.println("LIS3MDL Found!");

        mag_.reset();
        mag_.setRange(LIS3MDL_RANGE_16_GAUSS);
        mag_.setDataRate(MAG_DATARATE);
        mag_.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        mag_.setOperationMode(LIS3MDL_CONTINUOUSMODE);

        vPortEnterCritical();

        attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), magDaqISR, arduino::RISING);

        mag_.read(); // read once to clear interrupt

        vPortExitCritical();


    }

    void magDaqISR() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Send notification to control task on INDEX 0, unblocking the update(). 

        configASSERT( mag_taskHandle != NULL );
        vTaskNotifyGiveFromISR(mag_taskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    void magDaqThread(void*) {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
            if (xSemaphoreTake(sensors::IMU_main::i2c0_mutex_, 0) == pdTRUE) {
                mag_.read();
                magData_.x = mag_.x_gauss + MAG_X_OFFSET;
                magData_.y = mag_.y_gauss + MAG_Y_OFFSET;
                magData_.z = mag_.z_gauss + MAG_Z_OFFSET;
                xSemaphoreGive(magData_.ready);
    
                if (att_est_threads::updateTaskHandle_ != NULL ) {
                    xTaskNotifyGive(att_est_threads::updateTaskHandle_);
                }
            }
        }
    }

} // namespace mag