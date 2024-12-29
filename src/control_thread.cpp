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

    QuaternionPID attitudePID(attitude_dt_, maxRate_, minRate_, attKp_, attKi_, attKd_);
    RatePID ratePID(rate_dt_, maxServoPos, minServoPos, rateKp_, rateKi_, rateKd_);

    Eigen::Vector3d euler;
    sensor_msgs::GyroMsg current_gyro_data;

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        if (xSemaphoreTake(att_estimator__.ready, 0) == pdTRUE) {
            current_attitude_ = att_estimator__.newest_attitude_quat;
            xSemaphoreGive(att_estimator__.ready);
        }
        if (xSemaphoreTake(gyro_data__.ready, 0) == pdTRUE) {
            current_gyro_data = gyro_data__;
            xSemaphoreGive(gyro_data__.ready);
        }

        if (mcu_state_.load() == ControllerState::STBY || mcu_state_.load() == ControllerState::CALIBRATING) {

        } else {
            

            Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
            Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
            Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
            // Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
            euler = current_attitude_.to_euler();
            // Eigen::Vector3d euler = att_estimator__.integrated_quat.to_euler();

            Serial.println("Orientation: yaw: " + String(euler(0)) + " pitch: " + String(euler(1)) + " roll: " + String(euler(2)));
            // Serial.println("HeadingX: " + String(headingX));

            // call the pids to computer corrections
            attitudeOutput_ = attitudePID.compute(target_attitude_, current_attitude_);
            rateOutput_ = ratePID.compute(attitudeOutput_, gyro_data__.toVector() - gyro_data__.toBiasVector());  // x, y, z for servos
            tvc_mount__.move_mount(rateOutput_(1), rateOutput_(2));
            
            Serial.println();


            Serial.println("Attitude Output: " + String(attitudeOutput_(0)) + " " + String(attitudeOutput_(1)) + " " + String(attitudeOutput_(2)));
            Serial.println("Rate Output: " + String(rateOutput_(0)) + " " + String(rateOutput_(1)) + " " + String(rateOutput_(2)));

            Serial.println();
            Serial.println();
            
        }
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::CONTROL;
            Serial.println("Control loop delayed");
        }
    }

}

} // namespace controls_thread