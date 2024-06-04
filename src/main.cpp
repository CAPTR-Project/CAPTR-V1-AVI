/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: main.cpp
Auth: Alex Wang, Yubo Wang
Desc: Main file for MCU

*/

#include "Arduino.h"
#include "main.hpp"

#include "CAPTR_PIN_DRIVER.hpp"

ControllerState mcu_state = ControllerState::LV_ON;
ErrorState error_state = ErrorState::NONE;

bool new_state = true;
unsigned int loop_start;

void setup() {
  HwSetupPins();
  
  Serial.begin(115200);

  initBMP(BMP390_CHIP_ID, &Wire);
  initIMU(106U, &Wire);
}

void loop() {
  loop_start = micros();

  double altitude = bmp.readAltitude(1013.25);
  
  lsm_accel->getEvent(&accelMain);
  lsm_gyro->getEvent(&gyro);
  

  switch(mcu_state)
  {
    case ControllerState::LV_ON:

      if(new_state)
      {
        Serial.println("FSM: LV_ON");
        new_state = false;
      }

      // TODO: LV_ON Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::TVC_UP;
        new_state = true;
      }

      break;

    case ControllerState::TVC_UP:
      
      if(new_state)
      {
        Serial.println("FSM: TVC_UP");
        new_state = false;
      }

      // TODO: TVC_UP Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::TVC_UP;
      }

    case ControllerState::RECOVERY:
      
      if(new_state)
      {
        Serial.println("FSM: RECOVERY");
        new_state = false;
      }

      // TODO: Recovery Code

      if (error_state == ErrorState::NONE) {
        mcu_state = ControllerState::TVC_UP;
      }
  }
}

/*
=============================================
============ Function Definitions ===========
=============================================
*/

void initBMP(uint8_t i2cAddr, TwoWire* I2CBus){
  if (!bmp.begin_I2C(i2cAddr, &Wire)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP3 sensor found");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}

void initIMU(uint8_t i2cAddr, TwoWire* I2CBus) {
  if (!lsm.begin_I2C(i2cAddr, I2CBus)) {
    // if (!lsm.begin_SPI(LSM_CS)) {
    // if (!lsm.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS Found!");

  // Set to 2G range and 26 Hz update rate
  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm.setAccelDataRate(LSM6DS_RATE_833_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_833_HZ);

  lsm_accel = lsm.getAccelerometerSensor();
  lsm_accel->printSensorDetails();

  lsm_gyro = lsm.getGyroSensor();
  lsm_gyro->printSensorDetails();
}