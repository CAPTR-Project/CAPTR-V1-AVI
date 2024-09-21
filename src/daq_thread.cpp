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
    if (!sensors_lib::initMag(&mag__, 0x1E, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN)) {
        error_state_.store(ErrorState::MAG);
    }
    // mag_data__.ready = xSemaphoreCreateMutex();
    

    // if (!sensors_lib::initBMP(&bmp__, BMP3_ADDR_I2C_SEC, &Wire1, baroISR, BARO_INT_PIN)) {
    //     error_state_.store(ErrorState::BARO);
    // }

    bmp390_set_addr_pin(&bmp__, bmp390_address_t::BMP390_ADDRESS_ADO_HIGH);
    bmp390_set_interface(&bmp__, bmp390_interface_t::BMP390_INTERFACE_IIC);
    DRIVER_BMP390_LINK_INIT(&bmp__, bmp390_handle_t);
    DRIVER_BMP390_LINK_IIC_INIT(&bmp__, bmp390_interface_iic_init);
    DRIVER_BMP390_LINK_IIC_DEINIT(&bmp__, bmp390_interface_iic_deinit);
    DRIVER_BMP390_LINK_IIC_READ(&bmp__, bmp390_interface_iic_read);
    DRIVER_BMP390_LINK_IIC_WRITE(&bmp__, bmp390_interface_iic_write);
    DRIVER_BMP390_LINK_SPI_INIT(&bmp__, bmp390_interface_spi_init);
    DRIVER_BMP390_LINK_SPI_DEINIT(&bmp__, bmp390_interface_spi_deinit);
    DRIVER_BMP390_LINK_SPI_READ(&bmp__, bmp390_interface_spi_read);
    DRIVER_BMP390_LINK_SPI_WRITE(&bmp__, bmp390_interface_spi_write);
    DRIVER_BMP390_LINK_DELAY_MS(&bmp__, bmp390_interface_delay_ms);
    DRIVER_BMP390_LINK_DEBUG_PRINT(&bmp__, bmp390_interface_debug_print);

    int res = bmp390_init(&bmp__);
    
    if (res > 0) {
        error_state_.store(ErrorState::BARO);
        while (true) {
            Serial.println("BMP390 init failed: " + String(res));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // if (bmp390_interrupt_init(bmp390_interface_t::BMP390_INTERFACE_IIC, bmp390_address_t::BMP390_ADDRESS_ADO_LOW, baroISR)) {
    //     error_state_.store(ErrorState::BARO);
    // }
    bmp390_set_mode(&bmp__, bmp390_mode_t::BMP390_MODE_NORMAL_MODE);
    bmp390_set_odr(&bmp__, BARO_DATARATE);
    bmp390_set_temperature_oversampling(&bmp__, bmp390_oversampling_t::BMP390_OVERSAMPLING_x4);
    bmp390_set_pressure_oversampling(&bmp__, bmp390_oversampling_t::BMP390_OVERSAMPLING_x16);
    bmp390_set_filter_coefficient(&bmp__, bmp390_filter_coefficient_t::BMP390_FILTER_COEFFICIENT_7);

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
            baro_data__ = temp_baro_data_;
            baro_data_ready_ = false;
        }
    }
}

} // namespace daq_thread