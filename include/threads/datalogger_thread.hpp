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
#include "config.hpp"
#include <cmath>
#include <SerialFlash.h>

namespace datalogger_thread {

// Include libs

// ============================== Constants/Vars ==================================

// Include constants/variables
inline TaskHandle_t taskHandle = NULL;

// ============================ Structs/Enums/Unions ==============================

struct SensorLog {
    sensor_msgs::BaroMsg baro;
    sensor_msgs::AccelMsg accel;
    sensor_msgs::GyroMsg gyro;
    sensor_msgs::MagMsg mag;
    uint32_t timestamp;
};

// ============================ Function Prototypes ==============================


void datalogger_thread(void*);

// Validation function declarations
bool isValidAccelData(const sensor_msgs::AccelMsg& accel);
bool isValidGyroData(const sensor_msgs::GyroMsg& gyro);
bool isValidBaroData(const sensor_msgs::BaroMsg& baro);
bool isValidMagData(const sensor_msgs::MagMsg& mag);

}

#endif