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

    QuaternionPID attitudePID(attitude_dt_, maxRate_, minRate_, attKp_, attKi_, attKd_, attIntegClamp_, attAlpha_, attTau_);
    RatePID ratePID(rate_dt_, maxServoPos, minServoPos, rateKp_, rateKi_, rateKd_, rateIntegClamp_, attAlpha_, attTau_);

    Eigen::Vector3d euler;
    UnitQuaternion currentAttitude{1, 0, 0, 0};
    sensor_msgs::GyroMsg currentGyroData;
    sensor_msgs::AccelMsg currentAccelData;

    // UnitQuaternion current_attitude;
    while (true) {
        // Control loop
        currentAttitude = att_est_tasks::att_estimator_.newest_attitude_quat;

        currentGyroData = sensors::IMU_main::gyroData_;

        currentAccelData = sensors::IMU_main::accelData_;

        if (resetFlag_) {
            // reset the integrators
            attitudePID.reset();
            ratePID.reset();
            resetFlag_ = false;
        }

        // call the pids to compute corrections
        attitudeOutput_ = attitudePID.compute(targetAttitude_, currentAttitude);
        rateOutput_ = ratePID.compute(attitudeOutput_, currentGyroData.toVector() - currentGyroData.toBiasVector());  // z, y, x for servos
        tvcMount_.move_mount(-rateOutput_(1), -rateOutput_(0));
            
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/CONTROL_FREQUENCY))));
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