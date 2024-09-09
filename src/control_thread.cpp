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
    UnitQuaternion current_attitude;
    while (1) {
        // Control loop
        if (xSemaphoreTake(att_estimator__.ready, 0) == pdTRUE) {
            current_attitude = att_estimator__.newest_attitude_quat;
            xSemaphoreGive(att_estimator__.ready);
        }



        // Read setpoints
        // Calculate control outputs
        // Write control outputs
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000/CONTROL_FREQUENCY));
        if (!xWasDelayed) {
            error_state_ = ErrorState::CONTROL;
        }
    }
}

} // namespace controls_thread