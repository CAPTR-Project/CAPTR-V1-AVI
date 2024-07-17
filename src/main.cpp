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
  mcu_state = ControllerState::LV_ON;
  error_state = ErrorState::NONE;
  new_state = true;

  HwSetupPins();
  
  Serial.begin(115200);
  Serial.println("hi");

  att_est_mutex = xSemaphoreCreateMutex();

  xTaskCreate(att_est_predict_thread, "Attitude Predictor"  , 500, nullptr, 3, &attEstPredictTaskHandle);
  xTaskCreate(att_est_update_thread , "Attitude Updator"    , 500, nullptr, 3, &attEstUpdateTaskHandle);

  xTaskCreate(control_thread        , "Control"             , 500, nullptr, 2, &controlTaskHandle);
  xTaskCreate(telem_logger_thread   , "Telemetry Logger"    , 500, nullptr, 1, &telemLoggerTaskHandle);

  // attach interrupts
  

  // initBMP(BMP390_CHIP_ID, &Wire);
  // initIMU(106U, &Wire);
  event_last_timer = millis();
}

void loop() {
  loop_start = micros();

  // double altitude = bmp.readAltitude(1013.25);
  
  // lsm_accel->getEvent(&accelMain);
  // lsm_gyro->getEvent(&gyro);

  switch(mcu_state)
  {
    case ControllerState::LV_ON:

      if(new_state)
      {
        Serial.println("FSM: LV_ON");
        new_state = false;

        event_last_timer = millis();
      }

      // TODO: LV_ON Code
      if (millis() - event_last_timer > 5000) {
          mcu_state = ControllerState::LAUNCH_DETECT;
          new_state = true;
      }

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::LV_ON;
      }

      break;

    case ControllerState::CALIBRATING:
        
        if(new_state)
        {
          Serial.println("FSM: CALIBRATING");
          new_state = false;
          event_last_timer = millis();
        }


        break;

    case ControllerState::LAUNCH_DETECT:
      
      if(new_state)
      {
        Serial.println("FSM: LAUNCH_DETECT");
        new_state = false;
      }

      // TODO: TVC_UP Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::LAUNCH_DETECT;
      }

      break;

    case ControllerState::POWERED_ASCENT:
      
      if(new_state)
      {
        Serial.println("FSM: POWERED_ASCENT");
        new_state = false;
      }

      // TODO: TVC_UP Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::POWERED_ASCENT;
      }

      break;

    case ControllerState::COAST:
      
      if(new_state)
      {
        Serial.println("FSM: COAST");
        new_state = false;
      }

      // TODO: Recovery Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::COAST;
      }

      break;

    case ControllerState::RECOVERY:
      
      if(new_state)
      {
        Serial.println("FSM: RECOVERY");
        new_state = false;
      }

      // TODO: Recovery Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::RECOVERY;
      }

      break;
    
    case ControllerState::LANDED:
      
      if(new_state)
      {
        Serial.println("FSM: LANDED");
        new_state = false;
        // command dump data to SD card
      }

      // TODO: Recovery Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::LANDED;
      }

      break;

    default:
      Serial.println("FSM: ERROR");
      error_state = ErrorState::FSM;
      break;
  }
}

/*
=============================================
============ Function Definitions ===========
=============================================
*/
