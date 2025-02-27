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
    serial_port_mutex__ = xSemaphoreCreateMutex();

    // Initialize the serial port
    Serial.begin(0);

    HwSetupPins();

    xSemaphoreTake(serial_port_mutex__, portMAX_DELAY);
    Serial.println(PSTR("\r\nBooted FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));
    xSemaphoreGive(serial_port_mutex__);

    daq_threads::daq_start();

    xTaskCreate(state_mgmt_thread::state_mgmt_thread, 
                "FSM", 1000, nullptr, 0, &state_mgmt_thread::taskHandle);

    vTaskStartScheduler();

}

void loop() {
    
}