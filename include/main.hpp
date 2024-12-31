/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: main.hpp
Auth: Alex Wang, Yubo Wang
Desc: Header file for MCU

*/

// ================================== Includes ====================================
// ================================================================================

#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include "globals.hpp"
#include "config.hpp"

// functions
#include "daq_ISRs.hpp"

// CAPTR libs
#include "CAPTR_PIN_DRIVER.hpp"
#include "sensors_setup.hpp"
#include "attitude_estimator.hpp"
#include "quaternion.h"

// threads
#include "threads/attitude_est_thread.hpp"
#include "threads/control_thread.hpp"
#include "threads/daq_thread.hpp"
#include "threads/datalogger_thread.hpp"
#include "threads/state_mgmt_thread.hpp"

// tasks
#include "tasks/gyro_calib.hpp"
#include "tasks/mag_calib.hpp"
#include "tasks/orient_calib.hpp"