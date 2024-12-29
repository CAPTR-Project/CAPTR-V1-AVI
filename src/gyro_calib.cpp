/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: gyro_calib.cpp
Auth: Yubo Wang
Desc: Source file for MCU

*/

#include "tasks/gyro_calib.hpp"

namespace gyro_calib_task {

void gyroBiasEstimation_task(void*) {
    
    // Initialize variables
    gyro_calib_done = false;

    long cnt = 0;
    sensor_msgs::GyroMsg local_gyro_data;
    float x = 0;
    float y = 0;
    float z = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    TickType_t start_time = xTaskGetTickCount();

    while (xLastWakeTime < start_time + pdMS_TO_TICKS(GYRO_CALIBRATION_TIME)) {
        // Read gyro data
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (xSemaphoreTake(gyro_data__.ready, 1) == pdTRUE) {
            local_gyro_data = gyro_data__;
            xSemaphoreGive(gyro_data__.ready);
        
            x += local_gyro_data.x;
            y += local_gyro_data.y;
            z += local_gyro_data.z;
            cnt++;
        }
        xLastWakeTime = xTaskGetTickCount();
    }
    x = x / cnt;
    y = y / cnt;
    z = z / cnt;

    att_estimator__.set_gyroBiases(z, y, x);

    daq_threads::gyro_bias_x_ = x;
    daq_threads::gyro_bias_y_ = y;
    daq_threads::gyro_bias_z_ = z;

    gyro_calib_done = true;
    Serial.println("Gyro calibration done");

    vTaskDelete(NULL);
    return;
}

} // namespace gyro_calib_task