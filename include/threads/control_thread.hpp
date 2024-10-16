/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: control_thread.hpp
Auth: Yubo Wang
Desc: Header file for control thread

*/

#ifndef CONTROL_THREAD_HPP
#define CONTROL_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "config.hpp"
#include "globals.hpp"
#include "attitude_estimator.hpp"

namespace controls_thread {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;
Eigen::Quaternion4f _target_attitude{Eigen::Quaternion3d::Identity()}; // set member variable to be identity
Eigen::Quaternion4f _cur_attitude{Eigen::Quaternion3d::Identity()}; 
Eigen::Quaternion4f _error_attitude{Eigen::Quaternion3d::Identity()};
Eigen::Vector3f _error_vector{Eigen::Vector3d::Zero()};

Eigen::Vector3f _Kp{1.0, 0.0, 1.0};
Eigen::Vector3f _Kd{1.0, 0.0, 1.0};
float _u_pitch, _u_roll, _u_yaw;

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif