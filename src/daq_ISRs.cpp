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
    if (!imu__.readAcceleration(new_accel_data.x, new_accel_data.y, new_accel_data.z)) {
        error_state_ = ErrorState::ACCEL;
    }
    daq_thread::temp_accel_data_ = new_accel_data;
    daq_thread::accel_data_ready_ = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_thread::taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void gyroISR() {
    // update gyro data global variable
    
    sensor_msgs::GyroMsg new_gyro_data;
    if (!imu__.readGyroscope(new_gyro_data.x, new_gyro_data.y, new_gyro_data.z)) {
        error_state_ = ErrorState::GYRO;
    }
    daq_thread::temp_gyro_data_ = new_gyro_data;
    daq_thread::gyro_data_ready_ = true;

    // Send notification to control task on INDEX 0, unblocking the predict(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_thread::taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR() {
    // Baro
}

void magISR() {
    // update mag__ data global variable

    sensor_msgs::MagMsg new_mag_data;
    if (!mag__.readMagneticField(new_mag_data.x, new_mag_data.y, new_mag_data.z)) {
        error_state_ = ErrorState::MAG;
    }
    daq_thread::temp_mag_data_ = new_mag_data;
    daq_thread::mag_data_ready_ = true;
    
    // Send notification to control task on INDEX 0, unblocking the update(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_thread::taskHandle != NULL );
    vTaskNotifyGiveFromISR(daq_thread::taskHandle, &xHigherPriorityTaskWoken);
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