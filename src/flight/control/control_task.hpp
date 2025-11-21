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
inline Eigen::Vector3d actuatorOutputs_;

inline Eigen::Vector3d currentRates_{0, 0, 0};

inline double attitude_dt_ = (1.0 * CONTROL_OUTER_RATE_DIVISION) / CONTROL_FREQUENCY;
inline double rate_dt_ = 1.0 / CONTROL_FREQUENCY;

// gains in x, y, z
inline Eigen::Vector3d attKp_{10, 10, 10};  
inline Eigen::Vector3d attKi_{0, 0, 0};
inline Eigen::Vector3d attKd_{0, 0, 0};
inline Eigen::Vector3d attIntegClamp_{0.05, 0.05, 0.05};
inline float attAlpha_ = 1;
inline float attTau_ = 0;

inline Eigen::Vector3d rateKp_{0.035355, 0.035355, 0.035355};
inline Eigen::Vector3d rateKi_{0.01688, 0.01688, 0.01688};
inline Eigen::Vector3d rateKd_{0.00219, 0.00219, 0.00219};
inline Eigen::Vector3d rateIntegClamp_{LIMIT_PITCH / 4, LIMIT_YAW / 4, LIMIT_YAW / 4};
inline float rateAlpha_ = 0.3;
inline float rateTau_ = 0.01;

// TVC mount object
inline tvc_mount_lib::TVC_mount tvcMount_(SERVO_PITCH_PIN, SCALING_PITCH, OFFSET_PITCH, LIMIT_PITCH, SERVO_YAW_PIN, SCALING_YAW, OFFSET_YAW, LIMIT_YAW);

// ============================ Function Prototypes ==============================

void control_task(void*);

void zeroIntegrators();

}

#endif // CONTROL_TASK_HPP