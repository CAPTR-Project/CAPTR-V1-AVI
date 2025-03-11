/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: mag.hpp
Auth: Yubo Wang
Desc: Source file for magnetometer data acquisition

*/

namespace sensors::mag {

    void magInit() {

    }

    void magDaqISR() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Send notification to control task on INDEX 0, unblocking the update(). 

        configASSERT( mag_taskHandle != NULL );
        vTaskNotifyGiveFromISR(mag_taskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    void magDaqThread(void*) {
        
    }

} // namespace mag