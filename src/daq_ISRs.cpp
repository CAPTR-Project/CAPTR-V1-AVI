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
    if (!imu.readAcceleration(new_accel_data.x, new_accel_data.y, new_accel_data.z)) {
        error_state = ErrorState::ACCEL;
    }
    accel_data = new_accel_data;
}

void gyroISR() {
    // update gyro data global variable
    
    sensor_msgs::GyroMsg new_gyro_data;
    if (!imu.readGyroscope(new_gyro_data.x, new_gyro_data.y, new_gyro_data.z)) {
        error_state = ErrorState::GYRO;
    }
    gyro_data = new_gyro_data;

    // Send notification to control task on INDEX 0, unblocking the predict(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( attEstPredictTaskHandle != NULL );
    vTaskNotifyGiveFromISR(attEstPredictTaskHandle, &xHigherPriorityTaskWoken);
    if (gyroCalibTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(gyroCalibTaskHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR() {
    // Baro
}

void magISR() {
    // update mag data global variable

    sensor_msgs::MagMsg new_mag_data;
    if (!mag.readMagneticField(new_mag_data.x, new_mag_data.y, new_mag_data.z)) {
        error_state = ErrorState::MAG;
    }
    mag_data = new_mag_data;
    
    // Send notification to control task on INDEX 0, unblocking the update(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( attEstUpdateTaskHandle != NULL );
    vTaskNotifyGiveFromISR(attEstUpdateTaskHandle, &xHigherPriorityTaskWoken);
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