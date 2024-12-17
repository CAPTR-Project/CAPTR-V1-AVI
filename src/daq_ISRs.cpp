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
    if (!imu__.readAcceleration(daq_thread::temp_accel_data_.x, daq_thread::temp_accel_data_.y, daq_thread::temp_accel_data_.z)) {
        // error_state_ = ErrorState::ACCEL;
    } else {
        daq_thread::accel_data_ready_ = true;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        configASSERT( daq_thread::taskHandle != NULL );
        vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void gyroISR() {
    // update gyro data global variable
    if (!imu__.readGyroscope(daq_thread::temp_gyro_data_.x, daq_thread::temp_gyro_data_.y, daq_thread::temp_gyro_data_.z)) {
        error_state_ = ErrorState::GYRO;
    } else {
        daq_thread::gyro_data_ready_ = true;

        // Send notification to control task on INDEX 0, unblocking the predict(). 
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        configASSERT( daq_thread::taskHandle != NULL );
        vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void baroISR() {
    // Serial.println("Baro read");
    /* read temperature pressure */
    // if (bmp390_read_pressure(&bmp__, nullptr, (float *)&daq_thread::temp_baro_data_.pressure) != 0)
    // {
    //     Serial.println("bmp390: read temperature and pressure failed.");
    
    //     return;
    // }
    daq_thread::temp_baro_data_.alt_msl = bmp__.readAltitude(BARO_PRESSURE_ASL * 0.01);

    daq_thread::temp_baro_data_.alt_agl = daq_thread::temp_baro_data_.alt_msl - ground_altitude_offset_msl__;

    daq_thread::temp_baro_data_.pressure = bmp__.pressure;

    daq_thread::baro_data_ready_ = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_thread::taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void magISR() {
    // update mag__ data global variable
    mag__.read();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // sensor_msgs::MagMsg new_mag_data = sensor_msgs::MagMsg();
    // new_mag_data.x = mag__.x_gauss + MAG_X_OFFSET;
    // new_mag_data.y = mag__.y_gauss + MAG_Y_OFFSET;
    // new_mag_data.z = mag__.z_gauss + MAG_Z_OFFSET;

    daq_thread::temp_mag_data_.x = mag__.x_gauss + MAG_X_OFFSET;
    daq_thread::temp_mag_data_.y = mag__.y_gauss + MAG_Y_OFFSET;
    daq_thread::temp_mag_data_.z = mag__.z_gauss + MAG_Z_OFFSET;
    daq_thread::mag_data_ready_ = true;
    
    // Send notification to control task on INDEX 0, unblocking the update(). 

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