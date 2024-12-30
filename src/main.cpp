/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: main.cpp
Auth: Alex Wang, Yubo Wang
Desc: Source file for MCU

*/

#include "main.hpp"
// #include "Arduino.h"

uint64_t loop_start;

FLASHMEM __attribute__((noinline)) void setup()
{

    Serial.begin(0);

    HwSetupPins();

    Serial.println(PSTR("\r\nBooted FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));


    xTaskCreate(state_mgmt_thread::state_mgmt_thread, 
                "FSM", 1000, nullptr, 0, &state_mgmt_thread::taskHandle);
    // vTaskDelay(pdMS_TO_TICKS(10));

    vTaskStartScheduler();

}

void loop() {
    
}