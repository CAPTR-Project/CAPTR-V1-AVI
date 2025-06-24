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
#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

// ================================== Includes ====================================

#include "arduino_freertos.h"
#include "attitude_estimator.hpp"
#include "quaternion.h"
#include "QuaternionPID.hpp"
#include "RatePID.hpp"
#include "mount_lib.hpp"

#include "config.hpp"
#include "state_mgmt/state_manager_task.hpp"
#include "sensor_fusion/attitude_est_task.hpp"

namespace control {

// ================================= vars ====================================

inline TaskHandle_t taskHandle = NULL;

inline bool resetFlag_ = false; // flag to reset integrators

inline UnitQuaternion targetAttitude_{1, 0, 0, 0}; // no rotation
inline Eigen::Vector3d targetRate_{0, 0, 0};

inline double attitude_dt_ = 1.0/CONTROL_FREQUENCY;
inline double rate_dt_ = 1.0/CONTROL_FREQUENCY;

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
inline float attAlpha_ = 1;
inline float attTau_ = 0;

inline Eigen::Vector3d rateKp_{0.6, 0.6, 0.6};
inline Eigen::Vector3d rateKi_{1.8, 1.8, 1.8};
inline Eigen::Vector3d rateKd_{0.038, 0.038, 0.038};
inline Eigen::Vector3d rateIntegClamp_{maxServoPos / 4, maxServoPos / 4, maxServoPos / 4};
inline float rateAlpha_ = 0.9;
inline float rateTau_ = 0.01;

inline Eigen::Vector3d attitudeOutput_;
inline Eigen::Vector3d rateOutput_;

// TVC mount object
inline tvc_mount_lib::TVC_mount tvcMount_(SERVO_PITCH_PIN, SCALING_PITCH, OFFSET_PITCH, LIMIT_PITCH, SERVO_YAW_PIN, SCALING_YAW, OFFSET_YAW, LIMIT_YAW);

// ============================ Function Prototypes ==============================

void control_task(void*);

void zeroIntegrators();

}

#endif // CONTROL_TASK_HPP