/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: datalogger_thread.cpp
Auth: Joseph Mi
Desc: Source file datalogger thread of BaroMsg, AccelMsg, GyroMsg, and mag_data

*/

#include "threads/datalogger_thread.hpp"

void datalogger_thread(void*) {
    // Data logger
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    
    // Create a file for logging if not already created (use library)

    //handle file open error



    while (1) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        
        // Read sensor data
        long long currentTimestamp = msElapsed.load();

        SensorLog currentLog;
        currentLog.baro = baro_data;
        currentLog.accel = accel_data;
        currentLog.gyro = gyro_data;
        currentLog.mag = mag_data;
        currentLog.timestamp = currentTimestamp;

        // OR individually
        // sensor_msgs::BaroMsg local_baro_data = baro_data;
        // sensor_msgs::AccelMsg local_accel_data = accel_data;
        // sensor_msgs::GyroMsg local_gyro_data = gyro_data;
        // sensor_msgs::MagMsg local_mag_data = mag_data;

        // Log data to file (use library)

        if (xWasDelayed) {
            // Log error
            error_state = ErrorState::CONTROL;

            
        }
    }
}