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

#pragma once

#include <Arduino.h>
#include "arduino_freertos.h"

#include "state_mgmt/state_manager.hpp"
#include "sensors/sensors.hpp"

#include "config.hpp"

void setup();
