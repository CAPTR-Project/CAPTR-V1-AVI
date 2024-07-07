/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: attitude_est_thread.cpp
Auth: Yubo Wang
Desc: Source file for attitude estimation thread

*/

#include "threads/attitude_est_thread.hpp"

std::atomic_bool last_action_was_predict = false;

void att_est_predict_thread(void*) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        ulTaskNotifyTakeIndexed(0, 1, portMAX_DELAY);
        if (xSemaphoreTake(att_est_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { // TODO: change delay to match freq of gyro
            // run UKF predict
            last_action_was_predict = true;
            xSemaphoreGive(att_est_mutex);
        }
        // wait for mutex lock and run UKF predict
    }
}

void att_est_update_thread(void*) {
    // Attitude estimation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        xTaskNotifyWait(0, 1, 0, portMAX_DELAY);
        if (last_action_was_predict) {
            if (xSemaphoreTake(att_est_mutex, pdMS_TO_TICKS(5)) == pdTRUE) { // TODO: change delay to match freq of mag
                // run UKF update
                last_action_was_predict = false;
                xSemaphoreGive(att_est_mutex);
            }
        }
    }
}