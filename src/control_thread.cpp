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

    QuaternionPID attitudePID(attitude_dt_, maxRate_, minRate_, attKp_, attKi_, attKd_, attIntegClamp_, att_alpha_, att_tau_);
    RatePID ratePID(rate_dt_, maxServoPos, minServoPos, rateKp_, rateKi_, rateKd_, rateIntegClamp_, att_alpha_, att_tau_);

    Eigen::Vector3d euler;
    sensor_msgs::GyroMsg current_gyro_data;
    sensor_msgs::AccelMsg current_accel_data;

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        if (xSemaphoreTake(att_estimator__.ready, 1) == pdTRUE) {
            current_attitude_ = att_estimator__.newest_attitude_quat;
            xSemaphoreGive(att_estimator__.ready);
        }
        if (xSemaphoreTake(gyro_data__.ready, 1) == pdTRUE) {
            current_gyro_data = gyro_data__;
            xSemaphoreGive(gyro_data__.ready);
        }
        if (xSemaphoreTake(accel_data__.ready, 1) == pdTRUE) {
            current_accel_data = accel_data__;
            xSemaphoreGive(accel_data__.ready);
        }

        if (mcu_state_.load() == ControllerState::STBY || mcu_state_.load() == ControllerState::CALIBRATING) {

        } else {
            

            xSemaphoreTake(serial_port_mutex__, pdMS_TO_TICKS(10));
            // call the pids to computer corrections
            attitudeOutput_ = attitudePID.compute(target_attitude_, current_attitude_);
            rateOutput_ = ratePID.compute(attitudeOutput_, current_gyro_data.toVector() - current_gyro_data.toBiasVector());  // z, y, x for servos
            tvc_mount__.move_mount(-rateOutput_(1), -rateOutput_(0));
            
            Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
            Serial.println("Acceleration: " + String(current_accel_data.x) + " " + String(current_accel_data.y) + " " + String(current_accel_data.z));
            Serial.println("Gyroscope: " + String(current_gyro_data.x) + " " + String(current_gyro_data.y) + " " + String(current_gyro_data.z));
            // Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
            euler = current_attitude_.to_euler();

            Serial.println("Orientation: yaw: " + String(euler(0)) + " pitch: " + String(euler(1)) + " roll: " + String(euler(2)));
            // Serial.println("HeadingX: " + String(headingX));

            Serial.println();


            Serial.println("Attitude Output: " + String(attitudeOutput_(0)) + " " + String(attitudeOutput_(1)) + " " + String(attitudeOutput_(2)));
            Serial.println("Rate Output: " + String(rateOutput_(0)) + " " + String(rateOutput_(1)) + " " + String(rateOutput_(2)));

            Serial.println();
            Serial.println();
            xSemaphoreGive(serial_port_mutex__);
            
        }
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::CONTROL;
            xSemaphoreTake(serial_port_mutex__, 0);
            Serial.println("Control loop delayed");
            xSemaphoreTake(serial_port_mutex__, 0);
        }
    }

}

} // namespace controls_thread