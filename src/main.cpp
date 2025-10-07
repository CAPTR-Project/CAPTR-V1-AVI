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

void setup() {

    delay(2000); // wait for serial monitor to open

    Serial.println(PSTR("\r\nBooted FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));
    
    // radio_cdh::radioInit();
    // local_logging::localLoggingInit();
    
    // Initialize the sensors
    sensors::baro::baroInit();
    sensors::IMU_main::IMUInit();
    sensors::mag::magInit();
    // sensors::gps::gpsInit();
    
    xTaskCreate(state_manager::stateManagerTask, "State Manager", 2048, NULL, 9, &state_manager::taskHandle);
    
    vTaskStartScheduler();
}

void loop() {
}