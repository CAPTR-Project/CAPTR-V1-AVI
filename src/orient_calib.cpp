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

#include "tasks/orient_calib.hpp"

namespace orient_calib_task {

void startingOrientationEstimation_task(void*) {
    
    // Initialize variables
    orient_calib_done = false;

    long cnt = 0;
    sensor_msgs::AccelMsg local_accel_data;
    float x = 0;
    float y = 0;
    float z = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    TickType_t start_time = xTaskGetTickCount();

    while (xLastWakeTime < start_time + pdMS_TO_TICKS(ORIENTATION_CALIBRATION_TIME)) {
        // Read accel data
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
        if (xSemaphoreTake(accel_data__.ready, 5) == pdTRUE) {
            local_accel_data = accel_data__;
            xSemaphoreGive(accel_data__.ready);
        
            x += local_accel_data.x;
            y += local_accel_data.y;
            z += local_accel_data.z;
            cnt++;
        }
        xLastWakeTime = xTaskGetTickCount();
    }
    x = x / cnt;
    y = y / cnt;
    z = z / cnt;

    // Calculate starting orientation
    Eigen::Vector3d normalized = Eigen::Vector3d(x, y, z).normalized();
    Eigen::Vector3d up = Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d axis = up.cross(normalized);
    double angle = acos(up.dot(normalized) / (up.norm() * normalized.norm()));
    starting_orientation = UnitQuaternion::from_rotVec(angle * axis(0), angle * axis(1), angle * axis(2)).conjugate();

    orient_calib_done = true;

    xSemaphoreTake(serial_port_mutex__, portMAX_DELAY);
    Serial.println("Orientation calibration done");
    xSemaphoreGive(serial_port_mutex__);

    vTaskDelete(NULL);
    return;
}

} // namespace gyro_calib_task