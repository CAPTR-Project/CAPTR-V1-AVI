/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: launch_detect.cpp
Auth: Yubo Wang
Desc: Source file for launch detection task

*/

#include "launch_detect_task.hpp"

namespace launch_detect {

    void launch_detect_task(void*) {
        // Initialize the launch detection task
        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;

        // Initialize the launch detection variables
        bool launchDetected = false;

        // Main loop
        while (1) {
            // Check if the acceleration is above the threshold
            if (sensors::IMU_main::accelData_.x > 10) {
                // Check if the launch has been detected
                if (!launchDetected) {
                    // Set the launch detected flag
                    launchDetected = true;
                }
                else {
                    // Notify the state manager that the launch has been detected
                    state_manager::requestState(MCUState::LAUNCH_DETECT);
                    // Quit the task
                    break;
                }
            }

            // Delay the task
            xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
            if (!xWasDelayed) {
                // Set the error state
                // state_manager::setError(ErrorState::FSM);
                Serial.println("Launch detect task delayed");
            }
        }

        // Quit the task
        vTaskDelete(NULL);
        return;
    }

}