/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: main.hpp
Auth: Alex Wang, Yubo Wang
Desc: Header file for data acquisition threads

*/

// IMU, Gyro, Baro, Mag, GPS

#ifndef DAQ_THREADS_HPP
#define DAQ_THREADS_HPP

// ================================== Includes ====================================

#include "rtos_includes.hpp"
#include "globals.hpp"


// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void imuCB(void *params);
void gyroCB(void *params);
void baroCB(void *params);
void magCB(void *params);

void GPS_thread(void *params);

#endif