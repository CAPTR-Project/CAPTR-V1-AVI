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

    QuaternionPID attitudePID(attitude_dt_, maxRate_, minRate_, attKp_, attKi_, attKd_);
    RatePID ratePID(rate_dt_, maxServoPos, minServoPos, rateKp_, rateKi_, rateKd_);

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        // if (xSemaphoreTake(att_estimator__.ready, 0) == pdTRUE) {
        //     current_attitude = att_estimator__.newest_attitude_quat;
        //     xSemaphoreGive(att_estimator__.ready);
        // }

        if (mcu_state_.load() == ControllerState::STBY || mcu_state_.load() == ControllerState::CALIBRATING) {

        } else {

        if (imu__.getEvent(&accel, &gyro, &temp)) {
            // Serial.println("GyroX: " + String(gyro.gyro.x));
            // Serial.println("GyroY: " + String(gyro.gyro.y));
            // Serial.println("GyroZ: " + String(gyro.gyro.z));
            headingX += (gyro.gyro.x) * 0.02;
        }


        // Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
        // Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
        // Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
        // Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
        // Eigen::Vector3d euler = att_estimator__.newest_attitude_quat.to_euler();
        // Eigen::Vector3d euler = att_estimator__.integrated_quat.to_euler();

        // Serial.println("Orientation: x: " + String(euler(0)) + " y: " + String(euler(1)) + " z: " + String(euler(2)));
        // Serial.println("HeadingX: " + String(headingX));
        // Serial.println();

        // call the pids to computer corrections
        attitudeOutput_ = attitudePID.compute(target_attitude_, att_estimator__.newest_attitude_quat);
        rateOutput_ = ratePID.compute(attitudeOutput_, gyro_data__.toVector());  // x, y, z for servos

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::CONTROL;
            Serial.println("Control loop delayed");
        }
        }
    }

}

} // namespace controls_thread