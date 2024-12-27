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
#include "quaternion.h"
#include "QuaternionPID.hpp"
#include "RatePID.hpp"

namespace controls_thread {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;

Quaternion target_attitude_{1, 0, 0, 0}; // no rotation
Eigen::Vector3f target_rate_{0, 0, 0};

float attitude_dt_ = 1/CONTROL_FREQUENCY;
float rate_dt_ = 1/CONTROL_FREQUENCY;

// max and min values
inline float maxServoPos = 9;
inline float minServoPos = -9;

inline float maxRate_ = 20.0;
inline float minRate_ = -20.0;

// gains in x, y, z (see if we can get away with using same gains for the same axis)
inline Eigen::Vector3f attKp_{1.0, 1.0, 1.0};  
inline Eigen::Vector3f attKi_{0.5, 0.5, 0.5};
inline Eigen::Vector3f attKd_{1.0, 1.0, 1.0};

inline Eigen::Vector3f rateKp_{1.0, 1.0, 1.0};
inline Eigen::Vector3f rateKi_{0.5, 0.5, 0.5};
inline Eigen::Vector3f rateKd_{1.0, 1.0, 1.0};

// outputs
inline Eigen::Vector3f attitudeOutput_{0, 0, 0};
inline Eigen::Vector3f rateOutput_{0, 0, 0};

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif // CONTROL_THREAD_HPP