/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: state_manager.hpp
Auth: Yubo Wang
Desc: Source file for state manager thread
*/

#include "state_manager.hpp"

namespace state_manager {

    void state_manager_thread(void*) {
        stateQueue = xQueueCreate(10, sizeof(ControllerState));


        // Initialize the state manager
        currentState = ControllerState::STBY;
        errorState_ = ErrorState::NONE;
        newStateFlag_ = false;

        ControllerState newState;
        while (1)
        {
            if (xQueueReceive(stateQueue, &newState, portMAX_DELAY) == pdPASS)
            {
                if (newState != currentState) 
                {
                    
                    switch (newState)
                    {
                     case ControllerState::STBY:
                        // if calibrated, ...
                        break;
                    case ControllerState::CALIBRATING:
                        if (currentState == ControllerState::STBY) {
                            // run calibration routine

                        }
                        break;

                    case ControllerState::LAUNCH_DETECT:
                        if (currentState == ControllerState::STBY) {
                            // start launch detection subroutine
                            // start controller?
                            // if no calibration, don't allow launch detection
                        }
                        break;

                    case ControllerState::POWERED_ASCENT:
                        if (currentState == ControllerState::LAUNCH_DETECT) {
                            // run powered ascent
                        }
                        break;

                    case ControllerState::COAST:
                        if (currentState == ControllerState::POWERED_ASCENT) {
                            // coast
                        }
                        break;

                    case ControllerState::RECOVERY:
                        if (currentState == ControllerState::COAST) {
                            // recovery
                        }
                        break;

                    case ControllerState::LANDED:
                        if (currentState == ControllerState::RECOVERY) {
                            // landed
                        }
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }

    void request_state(ControllerState state) {
        xQueueSend(stateQueue, &state, portMAX_DELAY);
    }

} // namespace state_manager