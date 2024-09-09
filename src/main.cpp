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

unsigned int loop_start;

unsigned int last_state_change_ms = 0;

void setup()
{
    mcu_state_ = ControllerState::LV_ON;
    error_state_ = ErrorState::NONE;
    new_state_ = true;

    baro_data__ = sensor_msgs::BaroMsg();
    accel_data__ = sensor_msgs::AccelMsg();
    gyro_data__ = sensor_msgs::GyroMsg();
    mag_data__ = sensor_msgs::MagMsg();

    att_est_threads::att_est_mutex_ = xSemaphoreCreateMutex();

    HwSetupPins();

    Serial.println("Serial print working!");

    // init sensors
    sensors_lib::initIMU(&imu__, LSM6DS_I2CADDR_DEFAULT, &Wire, IMU_DATARATE, imuISR, ACCEL_INT_PIN, gyroISR, GYRO_INT_PIN);
    sensors_lib::initBMP(&bmp__, BMP3_ADDR_I2C_SEC, &Wire, baroISR, BARO_INT_PIN);
    sensors_lib::initMag(&mag__, LIS3MDL_I2CADDR_DEFAULT, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN);
    sensors_lib::initGPS(&gps__, &GPS_SERIAL_PORT, GPS_BAUDRATE);

    taskENTER_CRITICAL();
    xTaskCreate(att_est_threads::att_est_predict_thread, 
                "Attitude Predictor", 2000, nullptr, 8, &att_est_threads::predictTaskHandle_);
    xTaskCreate(att_est_threads::att_est_update_thread,
                "Attitude Updator", 2000, nullptr, 8, &att_est_threads::updateTaskHandle_);
    xTaskCreate(controls_thread::control_thread, 
                "Control", 2000, nullptr, 10, &controls_thread::taskHandle);
    xTaskCreate(daq_thread::daq_thread, 
                "Telemetry Logger", 1000, nullptr, 1, &daq_thread::taskHandle);
    xTaskCreate(daq_thread::telem_logger_DAQ_thread, 
                "Sensor DAQ", 1000, nullptr, 50, &daq_thread::taskHandle);
    xTaskCreate(controls_thread::control_thread, 
                "Control", 2000, nullptr, 8, &controls_thread::taskHandle);
    taskEXIT_CRITICAL();

    last_state_change_ms = millis();
}

void loop()
{
    loop_start = micros();

    // double altitude = bmp__.readAltitude(1013.25);

    // lsm_accel->getEvent(&accelMain);
    // lsm_gyro->getEvent(&gyro);
    Serial.println("Altitude: " + String(baro_data__.alt_agl) + "m");
    Serial.println("Acceleration: " + String(accel_data__.x) + " " + String(accel_data__.y) + " " + String(accel_data__.z));
    Serial.println("Gyroscope: " + String(gyro_data__.x) + " " + String(gyro_data__.y) + " " + String(gyro_data__.z));
    Serial.println("Magnetometer: " + String(mag_data__.x) + " " + String(mag_data__.y) + " " + String(mag_data__.z));
    Eigen::Vector3d euler = att_estimator__.newest_attitude_quat.to_euler();
    Serial.println("Orientation: x: " + String(euler[0]) + " y: " + String(euler[1]) + " z: " + String(euler[2]));

    switch (mcu_state_)
    {
    case ControllerState::LV_ON:

        if (new_state_)
        {
            Serial.println("FSM: LV_ON");
            new_state_ = false;

            last_state_change_ms = millis();
        }

        // TODO: LV_ON Code
        if (millis() - last_state_change_ms > 5000)
        {
            mcu_state_ = ControllerState::CALIBRATING;
            new_state_ = true;
            break;
        }

        if (error_state_ == ErrorState::NONE)
        {
            mcu_state_ = ControllerState::LV_ON;
        }

        break;

    case ControllerState::CALIBRATING:

        if (new_state_)
        {
            Serial.println("FSM: CALIBRATING");
            new_state_ = false;
            last_state_change_ms = millis();
            xTaskCreate(gyro_calib_task::gyroBiasEstimation_task, "Gyro Calibration", 2000, nullptr, 4, &gyro_calib_task::taskHandle);
        }

        if (!mag_calib_task::mag_calib_done && gyro_calib_task::gyro_calib_done)
        {
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
        }

        break;

    case ControllerState::LAUNCH_DETECT:

        if (new_state_)
        {
            Serial.println("FSM: LAUNCH_DETECT");
            new_state_ = false;
        }

        if (accel_data__.z > 0.5)
        {
            mcu_state_ = ControllerState::POWERED_ASCENT;
            new_state_ = true;
            break;
        }

        if (error_state_ == ErrorState::NONE)
        {
            mcu_state_ = ControllerState::LAUNCH_DETECT;
        }

        break;

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
        }

        break;

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
        }

        break;

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
        }

        break;

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
        }

        break;

    default:
        Serial.println("FSM: ERROR");
        error_state_ = ErrorState::FSM;
        break;
    }


}

/*
=============================================
============ Function Definitions ===========
=============================================
*/
