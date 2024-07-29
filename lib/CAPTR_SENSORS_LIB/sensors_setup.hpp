#ifndef SENSORS_SETUP_HPP
#define SENSORS_SETUP_HPP

#include <Wire.h>
#include <Arduino.h>
#include "arduino_freertos.h"

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>

namespace sensors_lib {
    bool initBMP(Adafruit_BMP3XX* bmp, uint8_t i2cAddr, TwoWire* I2CBus, void (*alt_isr)(), uint8_t isr_pin);

    bool initIMU(Adafruit_LSM6DS* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate, 
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin);

    bool initMag(Adafruit_LIS3MDL* lis_mag, uint8_t i2cAddr, TwoWire* I2CBus, lis3mdl_dataRate_t datarate, 
                 void (*mag_isr)(), uint8_t mag_isr_pin);
    
    void initRadio();
    
}

#endif