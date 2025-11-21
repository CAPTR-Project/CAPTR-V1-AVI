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
#ifndef STATE_MANAGER_TASK_HPP
#define STATE_MANAGER_TASK_HPP

#include "arduino_freertos.h"
#include <atomic>

#include "config.hpp"
#include "state_defs.hpp"
#include "sensors/sensors.hpp"
#include "sensor_fusion/attitude_est_task.hpp"
#include "preflight/calibration_worker.hpp"
#include "preflight/launch_detect_task.hpp"
#include "flight/ascent_task.hpp"
#include "flight/control/control_task.hpp"
#include "CDH/sd_logging_task.hpp"

namespace state_manager {

    // ================================= Constants ====================================

    // =============================== Variables ======================================
    inline TaskHandle_t taskHandle;

    inline QueueHandle_t stateQueue;

    inline MCUState currentState;
    inline ErrorState errorState_;

    inline bool newStateFlag_;

    // ============================ Function Prototypes ===============================
    void stateManagerTask(void*);

    void requestState(MCUState state);

    void setError(ErrorState error);

    MCUState getState();

    ErrorState getError();

} // namespace state_manager

#endif // STATE_MANAGER_TASK_HPP