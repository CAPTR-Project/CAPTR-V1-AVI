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

    float headingX = 0.0;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        // if (xSemaphoreTake(att_estimator__.ready, 0) == pdTRUE) {
        //     current_attitude = att_estimator__.newest_attitude_quat;
        //     xSemaphoreGive(att_estimator__.ready);
        // }

        if (mcu_state_.load() == ControllerState::STBY || mcu_state_.load() == ControllerState::CALIBRATING) {

        } else {

        headingX += (gyro.gyro.x) * pdTICKS_TO_MS(xTaskGetTickCount() - xLastWakeTime) / 1000.0;

        // Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
        // Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
        // Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
        // Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
        // Eigen::Vector3d euler = att_estimator__.newest_attitude_quat.to_euler();
        // Eigen::Vector3d euler = att_estimator__.integrated_quat.to_euler();

        // Serial.println("Orientation: x: " + String(euler(0)) + " y: " + String(euler(1)) + " z: " + String(euler(2)));
        Serial.println("HeadingX: " + String(headingX));


        Serial.println();
        }

    //     // Serial.println("Control loop");
    //     // current state
    //     _cur_attitude = att_estimator__.newest_attitude_quat;

    //     // calc error (calc is short for calculate btw chat)     
    //     _error_attitude = _target_attitude * _cur_attitude.inverse();
    //     // Write control outputs
    //     // get the vector components of the quaternion
    //     error_vector[0] = error_euler[1];   // pitch
    //     error_vector[1] = error_euler[2];   // roll
    //     error_vector[2] = error_euler[3];   // yaw

    //     // control law
    //     _u_pitch = _Kp[0] * error_vector[0] + _Kd[0] * gyro_data__.x;
    //     _u_roll = _Kp[1] * error_vector[1] + _Kd[1] * gyro_data__.y;
    //     // _u_yaw = _Kp[2] * error_vector[2] + _Kd[2] * gyro_data__.z;

    //     att_cmd_pitch = _u_pitch;
    //     att_cmd_roll = _u_roll;
    //     //att_cmd_yaw = _u_yaw;

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::CONTROL;
            Serial.println("Control loop delayed");
        }
    }

}

} // namespace controls_thread