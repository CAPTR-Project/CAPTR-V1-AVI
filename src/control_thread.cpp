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

void control_thread(void*) {
    // Control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    UnitQuaternion current_attitude;
    while (1) {
        // Control loop
        if (xSemaphoreTake(att_est_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            current_attitude = att_estimator.newest_attitude_quat;
            xSemaphoreGive(att_est_mutex);
        }

        // Read setpoints
        // Calculate control outputs
        // Write control outputs
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1.0/CONTROL_FREQUENCY));
        if (xWasDelayed) {
            error_state = ErrorState::CONTROL;
        }
    }
}