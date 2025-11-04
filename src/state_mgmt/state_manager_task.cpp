/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: state_manager.hpp
Auth: Yubo Wang
Desc: Source file for state manager thread
*/

#include "state_manager_task.hpp"

namespace state_manager {

    void stateManagerTask(void*) {
        stateQueue = xQueueCreate(10, sizeof(MCUState));
        Serial.println("State manager thread started");

        // radio_cdh::radioInit();
        // local_logging::localLoggingInit();
        
        // Initialize the sensors
        sensors::baro::baroInit();
        sensors::IMU_main::i2c0_mutex_ = xSemaphoreCreateMutex();
        sensors::IMU_main::IMUInit();
        sensors::mag::magInit();
        // sensors::gps::gpsInit();

        // Initialize the state manager
        currentState = MCUState::STBY;
        errorState_ = ErrorState::NONE;

        requestState(MCUState::CALIBRATING);

        MCUState newState;
        while (1)
        {
            if (xQueueReceive(stateQueue, &newState, portMAX_DELAY) == pdPASS)
            {
                if (newState != currentState) 
                {
                    
                    switch (newState)
                    {
                    case MCUState::STBY:
                        if (currentState == MCUState::CALIBRATING) {
                            // stop calibration routine, if not stopped
                            calibration_worker::stop_calibration();
                            currentState = MCUState::STBY;
                        }
                        break;
                    case MCUState::CALIBRATING:
                        if (currentState == MCUState::STBY) {
                            // run calibration routine
                            xTaskCreate(calibration_worker::calibrate_task, "Calibrate", 2048, NULL, 1, &calibration_worker::calibrate_taskHandle);
                            currentState = MCUState::CALIBRATING;
                            Serial.println("Calibration started");
                        }
                        break;
                    case MCUState::LAUNCH_DETECT:
                        if (currentState == MCUState::STBY || currentState == MCUState::CALIBRATING) {
                            if (!calibration_worker::calibration_done) {
                                // if calibration not done, don't allow launch detection
                                Serial.println("Calibration not done, cannot start launch detection");
                                break;
                            }
                            Serial.println("Starting launch detection");
                            att_est_tasks::att_estimator_.init(
                                calibration_worker::startingOrientation,
                                Eigen::Vector3d(calibration_worker::gyroBiasZ, calibration_worker::gyroBiasY, calibration_worker::gyroBiasX),
                                calibration_worker::mag_vec_,
                                Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity() * 0.01,
                                Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity() * 0.01
                            );
                            // start attitude estimation tasks
                            xTaskCreate(att_est_tasks::att_est_predict_thread, "Attitude Estimation", 2048, NULL, 5, &att_est_tasks::predictTaskHandle_);
                            xTaskCreate(att_est_tasks::att_est_update_thread, "Attitude Estimation", 2048, NULL, 5, &att_est_tasks::updateTaskHandle_);
                            // start launch detection task
                            // xTaskCreate(launch_detect::launch_detect_task, "Launch Detect", 2048, NULL, 5, &launch_detect::taskHandle);
                            // start controller
                            xTaskCreate(control::control_task, "Control", 2048, NULL, 5, &control::taskHandle);
                            currentState = MCUState::LAUNCH_DETECT;
                            Serial.println("Launch detection started");
                        }
                        break;

                    // case MCUState::POWERED_ASCENT:
                    //     if (currentState == MCUState::LAUNCH_DETECT) {
                    //         xTaskCreate(ascent::ascent_task, "Ascent", 2048, NULL, 5, &ascent::taskHandle);
                    //         // zero controller integral terms
                    //         control::zeroIntegrators();
                    //         currentState = MCUState::POWERED_ASCENT;
                    //         Serial.println("Ascent started");
                    //     }
                    //     break;

                    // case MCUState::COAST:
                    //     if (currentState == MCUState::POWERED_ASCENT) {
                    //         // coast
                    //         xTaskCreate(coast::coast_task, "Coast", 2048, NULL, 5, &coast::taskHandle);
                    //         currentState = MCUState::COAST;
                    //         Serial.println("Coast started");
                    //     }
                    //     break;

                    // case MCUState::RECOVERY:
                    //     if (currentState == MCUState::COAST) {
                    //         // recovery
                    //         xTaskCreate(flight::recovery_task, "Recovery", 2048, NULL, 5, &flight::taskHandle);
                    //         currentState = MCUState::RECOVERY;
                    //         Serial.println("Recovery initiated");
                    //     }
                    //     break;

                    // case MCUState::LANDED:
                    //     if (currentState == MCUState::RECOVERY) {
                    //         // landed
                    //         xTaskCreate(logging::sd_save_task, "SD Save", 2048, NULL, 9, &logging::taskHandle);
                    //         currentState = MCUState::LANDED;
                    //         Serial.println("Landed");
                    //     }
                    //     break;

                    default:
                        Serial.println("Unknown state requested: " + String(static_cast<int>(newState)));
                        // If an unknown state is requested, we can set it to STBY
                        setError(ErrorState::FSM);
                        break;
                    }
                }
            }
        }
    }

    void requestState(MCUState state) {
        Serial.println("State change requested: " + String(static_cast<int>(state)));
        xQueueSend(stateQueue, &state, portMAX_DELAY);
    }

    void setError(ErrorState error) {
        errorState_ = error;
        Serial.println("Error state set to: " + String(static_cast<int>(error)));
        requestState(MCUState::ERROR); // Set state to STBY on error
    }

    MCUState getState() {
        return currentState;
    }

    ErrorState getError() {
        return errorState_;
    }

} // namespace state_manager