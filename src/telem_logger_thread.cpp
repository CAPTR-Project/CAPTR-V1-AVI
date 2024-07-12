/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: telem_logger_thread.cpp
Auth: Yubo Wang
Desc: Source file for telemetry and logging thread

*/

#include "threads/telem_logger_thread.hpp"

void telem_logger_thread(void*) {
    // Telemetry and logging
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        // Telemetry and logging loop
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        if (!xWasDelayed) {
            // Log error
        }
    }
}