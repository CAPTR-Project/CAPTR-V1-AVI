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
inline double maxServoPos = 9 * M_PI / 180;
inline double minServoPos = -9 * M_PI / 180;

inline double maxRate_ = 3.0;
inline double minRate_ = -3.0;

// gains in x, y, z (see if we can get away with using same gains for every axis)
inline Eigen::Vector3d attKp_{20, 20, 20};  
inline Eigen::Vector3d attKi_{0, 0, 0};
inline Eigen::Vector3d attKd_{0, 0, 0};
inline Eigen::Vector3d attIntegClamp_{0, 0, 0};
inline float att_alpha_ = 1;
inline float att_tau_ = 0;

inline Eigen::Vector3d rateKp_{0.6, 0.6, 0.6};
inline Eigen::Vector3d rateKi_{1.8, 1.8, 1.8};
inline Eigen::Vector3d rateKd_{0.038, 0.038, 0.038};
inline Eigen::Vector3d rateIntegClamp_{maxServoPos / 4, maxServoPos / 4, maxServoPos / 4};
inline float rate_alpha_ = 0.9;
inline float rate_tau_ = 0.01;

// outputs
inline Eigen::Vector3d attitudeOutput_{0, 0, 0};
inline Eigen::Vector3d rateOutput_{0, 0, 0};

// ============================ Function Prototypes ==============================

void control_thread(void*);

}

#endif // CONTROL_THREAD_HPP