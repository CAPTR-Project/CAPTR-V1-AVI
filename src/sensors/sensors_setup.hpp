#pragma once


#include <Wire.h>
#include <Arduino.h>
#include "arduino_freertos.h"
#include "avr/io.h"
#include "avr/interrupt.h"

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
// #include <driver_bmp390.h>
// #include <driver_bmp390_interface.h>

namespace sensors_lib {
    // bool initBMP(bmp390_handle_s* bmp, bmp390_address_t i2cAddr, TwoWire* I2CBus, bmp390_odr_t datarate, void (*alt_isr)(), uint8_t isr_pin);
    bool initBMP(Adafruit_BMP3XX* bmp, uint8_t i2cAddr, TwoWire* I2CBus, uint8_t dataRate,
                 void (*baro_isr)(), uint8_t baro_isr_pin);

    bool initIMU_LSM6DSO32(Adafruit_LSM6DSO32* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate, 
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin);

    bool initIMU_LSM6DS(Adafruit_LSM6DS* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate, 
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin);

    bool initMag(Adafruit_LIS3MDL* lis_mag, uint8_t i2cAddr, TwoWire* I2CBus, lis3mdl_dataRate_t datarate, 
                 void (*mag_isr)(), uint8_t mag_isr_pin);
    
    void initRadio();

    void initGPS(TinyGPSPlus* gps, HardwareSerial* serialPort, uint32_t baudrate);
    
}
