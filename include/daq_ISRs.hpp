/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: daq_ISRs.hpp
Auth: Yubo Wang
Desc: Header file for data acquisition Interrupt Service Routines

*/

// IMU, Gyro, Baro, Mag, GPS

#ifndef DAQ_THREADS_HPP
#define DAQ_THREADS_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"


// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void imuISR();
void gyroISR();
void baroISR();
void magISR();

void GPS_thread();

#endif