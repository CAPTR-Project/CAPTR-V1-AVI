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
        // error_state_ = ErrorState::GYRO;
    } else {
        daq_thread::gyro_data_ready_ = true;

        // Send notification to control task on INDEX 0, unblocking the predict(). 
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        configASSERT( daq_thread::taskHandle != NULL );
        vTaskNotifyGiveFromISR( daq_thread::taskHandle, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void baroISR(uint8_t type) {
    switch (type)
    {
        case BMP390_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            break;
        }
        case BMP390_INTERRUPT_STATUS_FIFO_FULL :
        {
            break;
        }
        case BMP390_INTERRUPT_STATUS_DATA_READY :
        {
            Serial.println("Read temp and press");
            /* read temperature pressure */
            if (bmp390_interrupt_read((float *)nullptr, (float *)&daq_thread::temp_baro_data_.pressure) != 0)
            {
                Serial.println("bmp390: read temperature and pressure failed.");
           
                return;
            }
            daq_thread::baro_data_ready_ = true;

            
            break;
        }
        default :
        {
            break;
        }
    }
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