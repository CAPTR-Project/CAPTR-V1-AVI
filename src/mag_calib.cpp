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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        local_mag_data = mag_data_;
        x += local_mag_data.x;
        y += local_mag_data.y;
        z += local_mag_data.z;
        cnt++;
    }
    x = x / cnt;
    y = y / cnt;
    z = z / cnt;

    att_estimator.set_magVec(x, y, z);

    mag_calib_done = true;

    vTaskDelete(NULL);
}

} // namespace gyro_calib_task