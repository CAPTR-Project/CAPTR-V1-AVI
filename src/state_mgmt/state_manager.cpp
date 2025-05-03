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

#include "state_manager.hpp"

namespace state_manager {

    void state_manager_thread(void*) {
        stateQueue = xQueueCreate(10, sizeof(ControllerState));
        vTaskDelay(pdMS_TO_TICKS(200)); // wait for kernel to start
        Serial.println("State manager thread started");
        // Initialize the sensors

        sensors::baro::baroInit();
        sensors::IMU_main::IMUInit();
        sensors::mag::magInit();

        // Initialize the state manager
        currentState = ControllerState::STBY;
        errorState_ = ErrorState::NONE;
        newStateFlag_ = false;

        ControllerState newState;
        while (1)
        {
            if (xQueueReceive(stateQueue, &newState, portMAX_DELAY) == pdPASS)
            {
                if (newState != currentState) 
                {
                    
                    switch (newState)
                    {
                     case ControllerState::STBY:
                        if (currentState == ControllerState::CALIBRATING) {
                            // stop calibration routine, if not stopped
                            calibration_routine::stop_calibration();
                            // if calibrated, start attitude estimation
                            if (calibration_routine::calibration_done) {
                                xTaskCreate(attitude_estimator::attitude_estimation_task, "Attitude Estimation", 2048, NULL, 5, &attitude_estimator::taskHandle);
                            }
                            currentState = ControllerState::STBY;
                        }
                        break;
                    case ControllerState::CALIBRATING:
                        if (currentState == ControllerState::STBY) {
                            // run calibration routine
                            xTaskCreate(calibration_routine::calibrate_task, "Calibrate", 2048, NULL, 1, &calibration_routine::calibrate_taskHandle);
                            currentState = ControllerState::CALIBRATING;
                            Serial.println("Calibration started");
                        }
                        break;

                    case ControllerState::LAUNCH_DETECT:
                        if (currentState == ControllerState::STBY) {
                            if (!calibration_routine::calibration_done) {
                                // if calibration not done, don't allow launch detection
                                Serial.println("Calibration not done, cannot start launch detection");
                                break;
                            }
                            xTaskCreate(launch_detect::launch_detect_task, "Launch Detect", 2048, NULL, 5, &launch_detect::taskHandle);
                            currentState = ControllerState::LAUNCH_DETECT;
                            Serial.println("Launch detection started");
                        }
                        break;

                    case ControllerState::POWERED_ASCENT:
                        if (currentState == ControllerState::LAUNCH_DETECT) {
                            // xTaskCreate(flight::ascent_subroutine, "Ascent", 2048, NULL, 5, &flight::taskHandle);
                            currentState = ControllerState::POWERED_ASCENT;
                            Serial.println("Ascent started");
                        }
                        break;

                    case ControllerState::COAST:
                        if (currentState == ControllerState::POWERED_ASCENT) {
                            // coast
                            // xTaskCreate(flight::coast_subroutine, "Coast", 2048, NULL, 5, &flight::taskHandle);
                            currentState = ControllerState::COAST;
                            Serial.println("Coast started");
                        }
                        break;

                    case ControllerState::RECOVERY:
                        if (currentState == ControllerState::COAST) {
                            // recovery
                            // xTaskCreate(flight::recovery_subroutine, "Recovery", 2048, NULL, 5, &flight::taskHandle);
                            currentState = ControllerState::RECOVERY;
                            Serial.println("Recovery initiated");
                        }
                        break;

                    case ControllerState::LANDED:
                        if (currentState == ControllerState::RECOVERY) {
                            // landed
                            // xTaskCreate(logging::sd_save_task, "SD Save", 2048, NULL, 9, &logging::taskHandle);
                            currentState = ControllerState::LANDED;
                            Serial.println("Landed");
                        }
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }

    void request_state(ControllerState state) {
        xQueueSend(stateQueue, &state, portMAX_DELAY);
    }

} // namespace state_manager