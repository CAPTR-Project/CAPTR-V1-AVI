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

#include "tasks/gyro_calib.hpp"

void gyro_calibration(void*) {
    // Control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        // Control loop
        // Read sensor data
        // Read setpoints
        // Calculate control outputs
        // Write control outputs
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}