/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: datalogger_thread.hpp
Auth: Joseph Mi
Desc: Header file for datalogger thread of BaroMsg, AccelMsg, GyroMsg, and mag_data

*/

#ifndef DATALOGGER_THREAD_HPP
#define DATALOGGER_THREAD_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"

// Include libs

// ================================= Constants ====================================

// Include constants/variables

// ============================ Structs/Enums/Unions ==============================

struct SensorLog {
    sensor_msgs::BaroMsg baro;
    sensor_msgs::AccelMsg accel;
    sensor_msgs::GyroMsg gyro;
    sensor_msgs::MagMsg mag;
    long long timestamp;
};

// ============================ Function Prototypes ==============================

void datalogger_thread(void*);

#endif