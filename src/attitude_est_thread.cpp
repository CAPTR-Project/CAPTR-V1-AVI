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
std::mutex att_est_mutex;

void att_est_predict_thread(void*) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        xTaskNotifyWait(0, 1, 0, portMAX_DELAY);
        // wait for mutex lock and run UKF predict
    }
}

void att_est_update_thread(void*) {
    // Attitude estimation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {
        if (last_action_was_predict) {
            xTaskNotifyWait(0, 1, 0, portMAX_DELAY);
            // wait for mutex lock and run UKF update
        }
    }
}