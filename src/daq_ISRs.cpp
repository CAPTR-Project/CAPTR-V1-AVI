/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: daq_threads.cpp
Auth: Yubo Wang
Desc: Source file for data acquisition Interrupt Service Routines

*/

#include "daq_ISRs.hpp"

void imuISR() {
    sensor_msgs::AccelMsg new_accel_data;
    if (!imu_.readAcceleration(new_accel_data.x, new_accel_data.y, new_accel_data.z)) {
        error_state_ = ErrorState::ACCEL;
    }
    accel_data_ = new_accel_data;
    telem_logger_thread::accel_data_ready_ = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( telem_logger_thread::daqTaskHandle != NULL );
    vTaskNotifyGiveFromISR( telem_logger_thread::daqTaskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void gyroISR() {
    // update gyro data global variable
    
    sensor_msgs::GyroMsg new_gyro_data;
    if (!imu_.readGyroscope(new_gyro_data.x, new_gyro_data.y, new_gyro_data.z)) {
        error_state_ = ErrorState::GYRO;
    }
    gyro_data_ = new_gyro_data;
    telem_logger_thread::gyro_data_ready_ = true;

    // Send notification to control task on INDEX 0, unblocking the predict(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( att_est_threads::predictTaskHandle_ != NULL );
    vTaskNotifyGiveFromISR(att_est_threads::predictTaskHandle_, &xHigherPriorityTaskWoken);
    if (gyro_calib_task::taskHandle != NULL) {
        vTaskNotifyGiveFromISR(gyro_calib_task::taskHandle, &xHigherPriorityTaskWoken);
    }
    configASSERT( telem_logger_thread::daqTaskHandle != NULL );
    vTaskNotifyGiveFromISR( telem_logger_thread::daqTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR() {
    // Baro
}

void magISR() {
    // update mag_ data global variable

    sensor_msgs::MagMsg new_mag_data;
    if (!mag_.readMagneticField(new_mag_data.x, new_mag_data.y, new_mag_data.z)) {
        error_state_ = ErrorState::MAG;
    }
    mag_data_ = new_mag_data;
    telem_logger_thread::mag_data_ready_ = true;
    
    // Send notification to control task on INDEX 0, unblocking the update(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( att_est_threads::updateTaskHandle_ != NULL );
    vTaskNotifyGiveFromISR(att_est_threads::updateTaskHandle_, &xHigherPriorityTaskWoken);

    if (mag_calib_task::taskHandle != NULL) {
        vTaskNotifyGiveFromISR(mag_calib_task::taskHandle, &xHigherPriorityTaskWoken);
    }

    configASSERT( telem_logger_thread::daqTaskHandle != NULL );
    vTaskNotifyGiveFromISR(telem_logger_thread::daqTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPS_thread(void*) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        // GPS loop
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}