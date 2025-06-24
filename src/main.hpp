/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: main.hpp
Auth: Alex Wang, Yubo Wang
Desc: Header file for main.cpp
*/

#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include "arduino_freertos.h"

#include "state_mgmt/state_manager_task.hpp"
#include "CDH/radio_cdh_task.hpp"
#include "CDH/local_logging_task.hpp"

#include "config.hpp"

void setup();

void loop();

#endif // MAIN_HPP
