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
            // Check if the launch has been detected
            if (!launchDetected) {
                // Check if the acceleration is above the threshold
                if (sensors::IMU_main::accelData_.x > 10) {
                    // Set the launch detected flag
                    launchDetected = true;
                    // Notify the state manager that the launch has been detected
                    state_manager::request_state(ControllerState::LAUNCH_DETECT);
                    // Log the launch detection
                    
                    // Quit the task
                    break;
                }
            }

            // Delay the task
            xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000/FSM_FREQUENCY));
            if (!xWasDelayed) {
                // Set the error state
                state_manager::set_error(ErrorState::FSM);
                Serial.println("Launch detect task delayed");
            }
        }

        // Quit the task
        vTaskDelete(NULL);
        return;
    }

}