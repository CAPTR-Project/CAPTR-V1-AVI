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

    // Initialize the serial port
    Serial.begin(0);

    Serial.println(PSTR("\r\nBooted FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    // radio_cdh::radioInit();
    // local_logging::localLoggingInit();

    // Initialize the sensors
    sensors::baro::baroInit();
    sensors::IMU_main::IMUInit();
    sensors::mag::magInit();
    // sensors::gps::gpsInit();

    xTaskCreate(state_manager::stateManagerTask, "State Manager", 2048, NULL, 10, &state_manager::taskHandle);

    vTaskStartScheduler();
}

void loop() {
    
}