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

unsigned int loop_start;

FLASHMEM __attribute__((noinline)) void setup()
{

    Serial.begin(0);

    HwSetupPins();

    vTaskStartScheduler();

    Serial.println(PSTR("\r\nBooted FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(state_mgmt_thread::state_mgmt_thread, 
                "FSM", 1000, nullptr, 0, &state_mgmt_thread::taskHandle);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreate(att_est_threads::att_est_predict_thread, 
                "Attitude Predictor", 4000, nullptr, 8, &att_est_threads::predictTaskHandle_);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreate(att_est_threads::att_est_update_thread,
                "Attitude Updator", 2000, nullptr, 8, &att_est_threads::updateTaskHandle_);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreate(controls_thread::control_thread, 
                "Control", 2000, nullptr, 8, &controls_thread::taskHandle);
    xTaskCreate(daq_thread::daq_thread, 
                "Sensor DAQ", 1000, nullptr, 9, &daq_thread::taskHandle);
    // xTaskCreate(datalogger_thread::datalogger_thread, 
    //             "Telemetry Logger", 1000, nullptr, 7, &datalogger_thread::taskHandle);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

}

void loop() {
    
}