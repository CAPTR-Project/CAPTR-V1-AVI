/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: daq_threads.cpp
Auth: Yubo Wang
Desc: Source file for data acquisition Interrupt Service Routines

*/

#include "daq_ISRs.hpp"

void imuISR(void*) {

}

void gyroISR(void*) {
    // update gyro data global variable

    // TODO: Implement gyro data update
    
    // Send notification to control task on INDEX 0, unblocking the predict(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( attEstPredictTaskHandle != NULL );
    vTaskNotifyGiveFromISR(attEstPredictTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR(void*) {
    // Baro
}

void magISR(void*) {
    // update mag data global variable

    // TODO: Implement mag data update
    
    // Send notification to control task on INDEX 1, unblocking the update(). 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( attEstUpdateTaskHandle != NULL );
    vTaskNotifyGiveFromISR(attEstUpdateTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPS_thread(void*) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        // GPS loop
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}