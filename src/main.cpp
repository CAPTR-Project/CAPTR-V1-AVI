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

unsigned int event_last_timer = 0;

void setup() {
  mcu_state_ = ControllerState::LV_ON;
  error_state_ = ErrorState::NONE;
  new_state_ = true;

  baro_data_ = sensor_msgs::BaroMsg();
  accel_data_ = sensor_msgs::AccelMsg();
  gyro_data_ = sensor_msgs::GyroMsg();
  mag_data_ = sensor_msgs::MagMsg();

  att_est_threads::att_est_mutex_ = xSemaphoreCreateMutex();
  telem_logger_thread::reading_flag_ = xSemaphoreCreateMutex();


  HwSetupPins();
  
  Serial.begin(115200);
  Serial.println("hi");


  // init sensors
  sensors_lib::initIMU(&imu_, LSM6DS_I2CADDR_DEFAULT, &Wire, IMU_DATARATE, imuISR, ACCEL_INT_PIN, gyroISR, GYRO_INT_PIN);
  sensors_lib::initBMP(&bmp, BMP3_ADDR_I2C_SEC, &Wire, baroISR, BARO_INT_PIN);
  sensors_lib::initMag(&mag_, LIS3MDL_I2CADDR_DEFAULT, &Wire, MAG_DATARATE, magISR, MAG_INT_PIN);

  xTaskCreate(att_est_threads::att_est_predict_thread, "Attitude Predictor"  , 1500, nullptr, 8, &att_est_threads::predictTaskHandle_);
  xTaskCreate(att_est_threads::att_est_update_thread , "Attitude Updator"    , 1500, nullptr, 8, &att_est_threads::updateTaskHandle_);
  xTaskCreate(controls_thread::control_thread        , "Control"             , 1000, nullptr, 10, &controls_thread::taskHandle);
  xTaskCreate(telem_logger_thread::telem_logger_thread   , "Telemetry Logger"    , 1000, nullptr, 1, &telem_logger_thread::taskHandle);
  xTaskCreate(telem_logger_thread::telem_logger_DAQ_thread, "Telemetry Logger DAQ", 1000, nullptr, 2, &telem_logger_thread::daqTaskHandle);

  event_last_timer = millis();
}

void loop() {
  loop_start = micros();

  // double altitude = bmp.readAltitude(1013.25);
  
  // lsm_accel->getEvent(&accelMain);
  // lsm_gyro->getEvent(&gyro);

  switch(mcu_state_)
  {
    case ControllerState::LV_ON:

      if(new_state_)
      {
        Serial.println("FSM: LV_ON");
        new_state_ = false;

        event_last_timer = millis();
      }

      // TODO: LV_ON Code
      if (millis() - event_last_timer > 5000) {
          mcu_state_ = ControllerState::CALIBRATING;
          new_state_ = true;
          break;
      }

      if (error_state_ == ErrorState::NONE) {
        mcu_state_ = ControllerState::LV_ON;
      }

      break;

    case ControllerState::CALIBRATING:
        
        if(new_state_)
        {
          Serial.println("FSM: CALIBRATING");
          new_state_ = false;
          event_last_timer = millis();
          xTaskCreate(gyro_calib_task::gyroBiasEstimation_task, "Gyro Calibration"  , 500, nullptr, 4, &gyro_calib_task::taskHandle);
        }

        if (gyro_calib_task::gyro_calib_done) {
          xTaskCreate(mag_calib_task::magVectorEstimation_task, "Magnetometer Calibration"  , 500, nullptr, 4, &mag_calib_task::taskHandle);
        }

        if (mag_calib_task::mag_calib_done) {
          att_estimator.initialized = true;
          mcu_state_ = ControllerState::LAUNCH_DETECT;
          new_state_ = true;
          break;
        }

        if (error_state_ == ErrorState::NONE) {
          mcu_state_ = ControllerState::CALIBRATING;
        }

        break;

    case ControllerState::LAUNCH_DETECT:
      
      if(new_state_)
      {
        Serial.println("FSM: LAUNCH_DETECT");
        new_state_ = false;
      }

      if (accel_data_.z > 0.5) {
        mcu_state_ = ControllerState::POWERED_ASCENT;
        new_state_ = true;
        break;
      }

      if (error_state_ == ErrorState::NONE) {
        mcu_state_ = ControllerState::LAUNCH_DETECT;
      }

      break;

    case ControllerState::POWERED_ASCENT:
      
      if(new_state_)
      {
        Serial.println("FSM: POWERED_ASCENT");
        new_state_ = false;
      }

      // TODO: TVC_UP Code

      if (error_state_ == ErrorState::NONE) {
        mcu_state_ = ControllerState::POWERED_ASCENT;
      }

      break;

    case ControllerState::COAST:
      
      if(new_state_)
      {
        Serial.println("FSM: COAST");
        new_state_ = false;
      }

      // TODO: Recovery Code

      if (error_state_ == ErrorState::NONE) {
        mcu_state_ = ControllerState::COAST;
      }

      break;

    case ControllerState::RECOVERY:
      
      if(new_state_)
      {
        Serial.println("FSM: RECOVERY");
        new_state_ = false;
      }

      // TODO: Recovery Code

      if (error_state_ == ErrorState::NONE) {
        mcu_state_ = ControllerState::RECOVERY;
      }

      break;
    
    case ControllerState::LANDED:
      
      if(new_state_)
      {
        Serial.println("FSM: LANDED");
        new_state_ = false;
        // command dump data to SD card
      }

      // TODO: Recovery Code

      if (error_state_ == ErrorState::NONE) {
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
