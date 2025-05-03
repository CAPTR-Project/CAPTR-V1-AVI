/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: state_manager.hpp
Auth: Yubo Wang
Desc: Header file for state manager thread
*/
#pragma once

#include "arduino_freertos.h"
#include <atomic>

#include "config.hpp"
#include "state_defs.hpp"
#include "sensors/sensors.hpp"
#include "preflight/calibration_routine.hpp"
#include "preflight/launch_detect_task.hpp"

namespace state_manager {

    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline TaskHandle_t taskHandle;

    QueueHandle_t stateQueue;

    inline ControllerState currentState;
    inline ErrorState errorState_;

    inline bool newStateFlag_;

    // ============================ Function Prototypes ===============================
    void state_manager_thread(void*);

    void request_state(ControllerState state);

    void set_error(ErrorState error);

    ControllerState get_state();

    ErrorState get_error();

} // namespace state_manager