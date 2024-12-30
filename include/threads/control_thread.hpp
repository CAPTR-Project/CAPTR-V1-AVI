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

inline UnitQuaternion target_attitude_{1, 0, 0, 0}; // no rotation
inline Eigen::Vector3d target_rate_{0, 0, 0};

inline double attitude_dt_ = 1.0/CONTROL_FREQUENCY;
inline double rate_dt_ = 1.0/CONTROL_FREQUENCY;

inline UnitQuaternion current_attitude_{1, 0, 0, 0};

// max and min values
inline double maxServoPos = 9;
inline double minServoPos = -9;

inline double maxRate_ = 20.0;
inline double minRate_ = -20.0;

// gains in x, y, z (see if we can get away with using same gains for every axis)
inline Eigen::Vector3d attKp_{10.0, 10.0, 10.0};  
inline Eigen::Vector3d attKi_{0, 0, 0};
inline Eigen::Vector3d attKd_{0, 0, 0};
inline float attN_ = 300;

inline Eigen::Vector3d rateKp_{0.1, 0.1, 0.1};
inline Eigen::Vector3d rateKi_{0, 0, 0};
inline Eigen::Vector3d rateKd_{0, 0, 0};
inline float rateN_ = 300;

// outputs
inline Eigen::Vector3d attitudeOutput_{0, 0, 0};
inline Eigen::Vector3d rateOutput_{0, 0, 0};

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif // CONTROL_THREAD_HPP