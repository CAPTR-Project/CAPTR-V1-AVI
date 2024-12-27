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
#include "QUATERNIONS_LIB/include/quaternion.h"
#include "pid/pid.hpp"

namespace controls_thread {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;
Eigen::Quaternion4f target_attitude_{Eigen::Quaternion3d::Identity()}; // set member variable to be identity
Eigen::Quaternion4f curAttitude_{Eigen::Quaternion3d::Identity()}; 
Eigen::Quaternion4f errorAttitude_{Eigen::Quaternion3d::Identity()};
Eigen::Vector3f errorVector_{Eigen::Vector3d::Zero()};

inline Eigen::Vector3f _Kp{1.0, 0.0, 1.0};
inline Eigen::Vector3f _Kd{1.0, 0.0, 1.0};
inline float _u_pitch, _u_roll, _u_yaw;

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif