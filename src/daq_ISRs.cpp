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

void accelISR() {
    // if (!imu__.readAcceleration(daq_thread::temp_accel_data_.x, daq_thread::temp_accel_data_.y, daq_thread::temp_accel_data_.z)) {
    //     // error_state_ = ErrorState::ACCEL;
    // } else {
    //     daq_thread::accel_data_ready_ = true;

    //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //     configASSERT( daq_thread::taskHandle != NULL );
    //     vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);
        
    //     portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    // }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_threads::accel_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::accel_taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void gyroISR() {
    // update gyro data global variable
    // if (!imu__.readGyroscope(daq_thread::temp_gyro_data_.x, daq_thread::temp_gyro_data_.y, daq_thread::temp_gyro_data_.z)) {
    //     error_state_ = ErrorState::GYRO;
    // } else {
    //     daq_thread::gyro_data_ready_ = true;

    //     // Send notification to control task on INDEX 0, unblocking the predict(). 
    //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //     configASSERT( daq_thread::taskHandle != NULL );
    //     vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);

    //     portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    // }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send notification to control task on INDEX 0. 
    configASSERT( daq_threads::gyro_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::gyro_taskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR() {
    // Serial.println("Baro read");
    /* read temperature pressure */
    // if (bmp390_read_pressure(&bmp__, nullptr, (float *)&daq_thread::temp_baro_data_.pressure) != 0)
    // {
    //     Serial.println("bmp390: read temperature and pressure failed.");
    
    //     return;
    // }
    // daq_thread::temp_baro_data_.alt_msl = bmp__.readAltitude(BARO_PRESSURE_ASL * 0.01);

    // daq_thread::temp_baro_data_.alt_agl = daq_thread::temp_baro_data_.alt_msl - ground_altitude_offset_msl__;

    // daq_thread::temp_baro_data_.pressure = bmp__.pressure;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_threads::baro_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::baro_taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void magISR() {
    // update mag__ data global variable
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Send notification to control task on INDEX 0, unblocking the update(). 

    configASSERT( daq_threads::mag_taskHandle != NULL );
    vTaskNotifyGiveFromISR(daq_threads::mag_taskHandle, &xHigherPriorityTaskWoken);
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