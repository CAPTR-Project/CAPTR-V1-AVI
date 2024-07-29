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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        local_gyro_data = gyro_data;
        x += local_gyro_data.x;
        y += local_gyro_data.y;
        z += local_gyro_data.z;
        cnt++;
        // xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
    x = x / cnt;
    y = y / cnt;
    z = z / cnt;

    att_estimator.set_gyroBiases(x, y, z);

    gyro_calib_done = true;

    vTaskDelete(NULL);
}