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

// CAPTR libs
#include "CAPTR_PIN_DRIVER.hpp"

#include "daq_ISRs.hpp"

// threads
#include "threads/attitude_est_thread.hpp"
#include "threads/control_thread.hpp"
#include "threads/telem_logger_thread.hpp"
