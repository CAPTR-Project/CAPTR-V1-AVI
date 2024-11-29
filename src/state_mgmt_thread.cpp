#include "threads/state_mgmt_thread.hpp"

namespace state_mgmt_thread {

void state_mgmt_thread(void*) {


    mcu_state_.store(ControllerState::LV_ON);
    error_state_.store(ErrorState::NONE);
    new_state_ = true;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;

    last_state_change_ms = millis();

    new_state_ = true;

    // error_state_ = ErrorState::NONE;

    while (true) {
    
        switch (mcu_state_)
        {
        case ControllerState::LV_ON:

            if (new_state_)
            {
                Serial.println("FSM: LV_ON");
                new_state_ = false;

                last_state_change_ms = millis();
            }

            // Serial.println("FSM: LV_ON");

            // TODO: LV_ON Code
            if (millis() - last_state_change_ms > 2000)
            {
                mcu_state_ = ControllerState::CALIBRATING;
                new_state_ = true;
                break;
            }

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::LV_ON;
                break;
            }


        case ControllerState::CALIBRATING:

            if (new_state_)
            {
                Serial.println("FSM: CALIBRATING");
                new_state_ = false;
                last_state_change_ms = millis();
                xTaskCreate(gyro_calib_task::gyroBiasEstimation_task, "Gyro Calibration", 2000, nullptr, 4, &gyro_calib_task::taskHandle);
                xTaskCreate(mag_calib_task::magVectorEstimation_task, "Magnetometer Calibration", 2000, nullptr, 4, &mag_calib_task::taskHandle);

            }

            if (mag_calib_task::mag_calib_done && gyro_calib_task::gyro_calib_done)
            {
                att_estimator__.initialized = true;
                mcu_state_ = ControllerState::LAUNCH_DETECT;
                new_state_ = true;
                break;
            }

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::CALIBRATING;
                break;
            }


        case ControllerState::LAUNCH_DETECT:

            if (new_state_)
            {
                Serial.println("FSM: LAUNCH_DETECT");
                new_state_ = false;
            }

            // Serial.println("FSM: LAUNCH_DETECT");

            if (accel_data__.z > 0.5)
            {
                mcu_state_ = ControllerState::POWERED_ASCENT;
                new_state_ = true;
                break;
            }

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::LAUNCH_DETECT;
                break;
            }


        case ControllerState::POWERED_ASCENT:

            if (new_state_)
            {
                Serial.println("FSM: POWERED_ASCENT");
                new_state_ = false;
            }

            // TODO: TVC_UP Code

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::POWERED_ASCENT;
                break;
            }


        case ControllerState::COAST:

            if (new_state_)
            {
                Serial.println("FSM: COAST");
                new_state_ = false;
            }

            // TODO: Recovery Code

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::COAST;
                break;
            }


        case ControllerState::RECOVERY:

            if (new_state_)
            {
                Serial.println("FSM: RECOVERY");
                new_state_ = false;
            }

            // TODO: Recovery Code

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::RECOVERY;
                break;

            }


        case ControllerState::LANDED:

            if (new_state_)
            {
                Serial.println("FSM: LANDED");
                new_state_ = false;
                // command dump data to SD card
            }

            // TODO: Recovery Code

            if (error_state_ == ErrorState::NONE)
            {
                mcu_state_ = ControllerState::LANDED;
                break;

            }


        default:
            Serial.println("FSM: ERROR");
            String error_state_str = String(int(error_state_.load()));
            Serial.println("Error state: " + error_state_str);
            break;
        }

        // taskYIELD();

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/FSM_FREQUENCY))));
        if (!xWasDelayed) {
            // error_state_ = ErrorState::FSM;
            Serial.println("FSM loop delayed");
        }
    }
}

} // namespace state_mgmt_thread