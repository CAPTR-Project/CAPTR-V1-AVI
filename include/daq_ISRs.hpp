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

// threads
#include "threads/attitude_est_thread.hpp"
#include "threads/control_thread.hpp"
#include "threads/telem_logger_thread.hpp"

// tasks
#include "tasks/gyro_calib.hpp"
#include "tasks/mag_calib.hpp"

// ================================= Constants ====================================

// ============================ Function Prototypes ==============================

void imuISR();
void gyroISR();
void baroISR();
void magISR();

void GPS_thread();

#endif