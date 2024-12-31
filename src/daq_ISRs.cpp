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

void accelISR() {
    // Serial.println("Accel ISR");

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_threads::accel_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::accel_taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void gyroISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send notification to control task on INDEX 0. 
    configASSERT( daq_threads::gyro_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::gyro_taskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void baroISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( daq_threads::baro_taskHandle != NULL );
    vTaskNotifyGiveFromISR( daq_threads::baro_taskHandle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void magISR() {
    // update mag__ data global variable
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Send notification to control task on INDEX 0, unblocking the update(). 

    configASSERT( daq_threads::mag_taskHandle != NULL );
    vTaskNotifyGiveFromISR(daq_threads::mag_taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}