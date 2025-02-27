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

#include "tasks/mag_calib.hpp"

namespace mag_calib_task {

void magVectorEstimation_task(void*) {
    
    // Initialize variables
    mag_calib_done = false;

    long cnt = 0;
    sensor_msgs::MagMsg local_mag_data;
    float x = 0;
    float y = 0;
    float z = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    TickType_t start_time = xTaskGetTickCount();
    while (xLastWakeTime < start_time + pdMS_TO_TICKS(GYRO_CALIBRATION_TIME)) {
        // Read gyro data
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (xSemaphoreTake(mag_data__.ready, 1) == pdTRUE) {
            local_mag_data = mag_data__;
            xSemaphoreGive(mag_data__.ready);
        
            x += local_mag_data.x;
            y += local_mag_data.y;
            z += local_mag_data.z;
            cnt++;
        }
        xLastWakeTime = xTaskGetTickCount();
    }
    x = x / cnt;
    y = y / cnt;
    z = z / cnt;

    att_estimator__.set_magVec(x, y, z);

    mag_calib_done = true;

    xSemaphoreTake(serial_port_mutex__, portMAX_DELAY);
    Serial.println("Mag calibration done");
    xSemaphoreGive(serial_port_mutex__);

    vTaskDelete(NULL);
    return;
}

} // namespace gyro_calib_task