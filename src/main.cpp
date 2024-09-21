/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████
██      ██   ██ ██         ██    ██   ██
 ██████ ██   ██ ██         ██    ██   ██

File: main.cpp
Auth: Alex Wang, Yubo Wang
Desc: Source file for MCU

*/

#include "main.hpp"
// #include "Arduino.h"

unsigned int loop_start;

static void task2(void*) {
    Serial.begin(0);
    while (true) {
        Serial.println("TICK");
        vTaskDelay(pdMS_TO_TICKS(1'000));

        Serial.println("TOCK");
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }
}

FLASHMEM __attribute__((noinline)) void setup()
{

    Serial.begin(0);
    
    mcu_state_.store(ControllerState::LV_ON);
    error_state_.store(ErrorState::NONE);
    new_state_ = true;

    baro_data__ = sensor_msgs::BaroMsg();
    accel_data__ = sensor_msgs::AccelMsg();
    gyro_data__ = sensor_msgs::GyroMsg();
    mag_data__ = sensor_msgs::MagMsg();

    att_estimator__.init(UnitQuaternion(1, 0, 0, 0),
                        Eigen::Vector3d(0, 0, 0),
                        Eigen::Vector3d(0, 0, 0),
                        Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity(),
                        Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity());

    att_est_threads::att_est_mutex_ = xSemaphoreCreateMutex();

    HwSetupPins();

    // xTaskCreate(att_est_threads::att_est_predict_thread, 
    //             "Attitude Predictor", 2000, nullptr, 8, &att_est_threads::predictTaskHandle_);
    // xTaskCreate(att_est_threads::att_est_update_thread,
    //             "Attitude Updator", 2000, nullptr, 8, &att_est_threads::updateTaskHandle_);
    xTaskCreate(controls_thread::control_thread, 
                "Control", 2000, nullptr, 8, &controls_thread::taskHandle);
    xTaskCreate(daq_thread::daq_thread, 
                "Sensor DAQ", 1000, nullptr, 9, &daq_thread::taskHandle);
    // xTaskCreate(datalogger_thread::datalogger_thread, 
    //             "Telemetry Logger", 1000, nullptr, 7, &datalogger_thread::taskHandle);
    xTaskCreate(state_mgmt_thread::state_mgmt_thread, 
                "FSM", 1000, nullptr, 0, &state_mgmt_thread::taskHandle);

    // xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);


    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop()
{
    // loop_start = micros();
    
    // Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
    // Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
    // Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
    // Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
    // Eigen::Vector3d euler = att_estimator__.newest_attitude_quat.to_euler();
    // Serial.println("Orientation: x: " + String(euler[0]) + " y: " + String(euler[1]) + " z: " + String(euler[2]));

    // switch (mcu_state_)
    // {
    // case ControllerState::LV_ON:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: LV_ON");
    //         new_state_ = false;

    //         last_state_change_ms = millis();
    //     }

    //     // TODO: LV_ON Code
    //     if (millis() - last_state_change_ms > 5000)
    //     {
    //         mcu_state_ = ControllerState::CALIBRATING;
    //         new_state_ = true;
    //         break;
    //     }

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::LV_ON;
    //     }

    //     break;

    // case ControllerState::CALIBRATING:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: CALIBRATING");
    //         new_state_ = false;
    //         last_state_change_ms = millis();
    //         xTaskCreate(gyro_calib_task::gyroBiasEstimation_task, "Gyro Calibration", 2000, nullptr, 4, &gyro_calib_task::taskHandle);
    //     }

    //     if (!mag_calib_task::mag_calib_done && gyro_calib_task::gyro_calib_done)
    //     {
    //         xTaskCreate(mag_calib_task::magVectorEstimation_task, "Magnetometer Calibration", 2000, nullptr, 4, &mag_calib_task::taskHandle);
    //     }

    //     if (mag_calib_task::mag_calib_done && gyro_calib_task::gyro_calib_done)
    //     {
    //         att_estimator__.initialized = true;
    //         mcu_state_ = ControllerState::LAUNCH_DETECT;
    //         new_state_ = true;
    //         break;
    //     }

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::CALIBRATING;
    //     }

    //     break;

    // case ControllerState::LAUNCH_DETECT:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: LAUNCH_DETECT");
    //         new_state_ = false;
    //     }

    //     if (accel_data__.z > 0.5)
    //     {
    //         mcu_state_ = ControllerState::POWERED_ASCENT;
    //         new_state_ = true;
    //         break;
    //     }

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::LAUNCH_DETECT;
    //     }

    //     break;

    // case ControllerState::POWERED_ASCENT:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: POWERED_ASCENT");
    //         new_state_ = false;
    //     }

    //     // TODO: TVC_UP Code

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::POWERED_ASCENT;
    //     }

    //     break;

    // case ControllerState::COAST:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: COAST");
    //         new_state_ = false;
    //     }

    //     // TODO: Recovery Code

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::COAST;
    //     }

    //     break;

    // case ControllerState::RECOVERY:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: RECOVERY");
    //         new_state_ = false;
    //     }

    //     // TODO: Recovery Code

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::RECOVERY;
    //     }

    //     break;

    // case ControllerState::LANDED:

    //     if (new_state_)
    //     {
    //         Serial.println("FSM: LANDED");
    //         new_state_ = false;
    //         // command dump data to SD card
    //     }

    //     // TODO: Recovery Code

    //     if (error_state_ == ErrorState::NONE)
    //     {
    //         mcu_state_ = ControllerState::LANDED;
    //     }

    //     break;

    // default:
    //     Serial.println("FSM: ERROR");
    //     error_state_ = ErrorState::FSM;
    //     break;
    // }

}

/*
=============================================
============ Function Definitions ===========
=============================================
*/
