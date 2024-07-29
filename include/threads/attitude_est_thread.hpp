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

// ================================= Constants ====================================

// =============================== Variables ======================================
inline std::atomic<bool> last_action_was_predict;

// ============================ Function Prototypes ===============================

void att_est_predict_thread(void*);
void att_est_update_thread(void*);

#endif