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

#include "control_task.hpp"

namespace control {

void control_task(void*) {
    // Control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;

    QuaternionPID attitudePID(attitude_dt_, MAX_CTRL_ATT_RATE, -MAX_CTRL_ATT_RATE, 
                              Eigen::Vector3d::Ones(), attKp_, attKi_, attKd_,
                              attIntegClamp_, attAlpha_, attTau_);
    RatePID ratePID(rate_dt_, LIMIT_PITCH, -LIMIT_PITCH, Eigen::Vector3d::Ones(), rateKp_, rateKi_, rateKd_, rateIntegClamp_, rateAlpha_, rateTau_);

    Eigen::Vector3d euler;
    UnitQuaternion currentAttitude{1, 0, 0, 0};
    sensor_msgs::GyroMsg currentGyroData;
    sensor_msgs::AccelMsg currentAccelData;

    double emaAlpha = 2 * M_PI * RATE_EMA_CUTOFF / (2 * M_PI * RATE_EMA_CUTOFF + CONTROL_FREQUENCY);

    uint32_t loop_cnt = 0;

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        currentAttitude = att_est_tasks::att_estimator_.newest_attitude_quat;

        currentGyroData = sensors::IMU_main::gyroData_;

        currentAccelData = sensors::IMU_main::accelData_;

        // apply EMA filter to gyro data
        currentRates_ = emaAlpha * (currentGyroData.toVector() - att_est_tasks::att_estimator_.x_hat_.block<3, 1>(4, 0)) + 
                        (1 - emaAlpha) * currentRates_;

        if (resetFlag_) { // integrator reset
            attitudePID.reset();
            ratePID.reset();
            resetFlag_ = false;
        }
        
        // call the pids to compute corrections
        if (loop_cnt % CONTROL_OUTER_RATE_DIVISION == 0) {
            euler = currentAttitude.to_euler();
            Serial.println("Current quaternion: " + String(currentAttitude.s) + ", " + String(currentAttitude.v_1) + ", " + String(currentAttitude.v_2) + ", " + String(currentAttitude.v_3));
            Serial.println("Current Attitude: yaw" + String(euler(0) * 180 / M_PI) + ", pitch" + String(euler(1) * 180 / M_PI) + ", roll" + String(euler(2) * 180 / M_PI));
            Serial.println("Current Gyro: x" + String(currentGyroData.x) + ", y" + String(currentGyroData.y) + ", z" + String(currentGyroData.z));
            Serial.println("Current Magnetometer: x" + String(sensors::mag::magData_.x) + ", y" + String(sensors::mag::magData_.y) + ", z" + String(sensors::mag::magData_.z));

            targetRate_ = attitudePID.compute(targetAttitude_, currentAttitude);
            loop_cnt = 0;
        }
        loop_cnt++;

        actuatorOutputs_ = ratePID.compute(targetRate_, currentRates_);  // z, y, x for servos
        tvcMount_.move_mount(-actuatorOutputs_(1), -actuatorOutputs_(0));

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(1000 / CONTROL_FREQUENCY)));
        if (!xWasDelayed) {
            // state_manager::setError(ErrorState::CONTROL);
            Serial.println("Control loop delayed");
        }
    }

}

void zeroIntegrators() {
    resetFlag_ = true;
}

} // namespace control_task