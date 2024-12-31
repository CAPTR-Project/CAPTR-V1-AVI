/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: attitude_est_thread.hpp
Auth: Yubo Wang
Desc: Header file for Attitude Estimation Thread

*/

#ifndef ATT_EST_THREAD_HPP
#define ATT_EST_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"
#include "config.hpp"

#include "attitude_estimator.hpp"

namespace att_est_threads {

// ================================= Constants ====================================

// =============================== Variables ======================================
inline TaskHandle_t predictTaskHandle_ = NULL;
inline TaskHandle_t updateTaskHandle_ = NULL;

inline SemaphoreHandle_t att_est_mutex_ = NULL;

inline std::atomic<bool> last_action_was_predict;

inline uint64_t current_time_us_;

// ============================ Function Prototypes ===============================

void att_est_predict_thread(void*);
void att_est_update_thread(void*);

} // namespace att_est_threads

#endif