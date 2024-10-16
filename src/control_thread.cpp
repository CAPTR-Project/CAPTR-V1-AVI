/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: control_thread.cpp
Auth: Yubo Wang
Desc: Source file for MCU

*/

#include "threads/control_thread.hpp"

namespace controls_thread {

void control_thread(void*) {
    // Control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        // if (xSemaphoreTake(att_estimator__.ready, 0) == pdTRUE) {
        //     current_attitude = att_estimator__.newest_attitude_quat;
        //     xSemaphoreGive(att_estimator__.ready);
        // }

        Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
        Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
        Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
        Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
        Eigen::Vector3d euler = att_estimator__.newest_attitude_quat.to_euler();
        Serial.println("Orientation: x: " + String(euler[0]) + " y: " + String(euler[1]) + " z: " + String(euler[2]));
        Serial.println();

        // Serial.println("Control loop");

        // Read setpoints
        // Calculate control outputs
        // Write control outputs
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::CONTROL;
            Serial.println("Control loop delayed");
        }
    }

}

} // namespace controls_thread