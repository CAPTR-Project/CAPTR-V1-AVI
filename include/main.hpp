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

// CAPTR libs
#include "CAPTR_PIN_DRIVER.hpp"

// threads
#include "threads/attitude_est_thread.hpp"
#include "threads/daq_threads.hpp"
#include "threads/control_thread.hpp"
#include "threads/telem_logger_thread.hpp"