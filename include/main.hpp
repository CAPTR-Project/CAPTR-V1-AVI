#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

//Definitions


// Variables
Adafruit_BMP3XX bmp; // barometer
Adafruit_LSM6DS3TRC lsm; // IMU
Adafruit_Sensor *lsm_accel, *lsm_gyro;
sensors_event_t accelMain, gyro;
// GPS
RH_RF95 rf95; // Radio
// SD Card
// Flash

// put function declarations here:

/**
 * @brief Initializes the BMP sensor on an I2C bus
 * 
 * @param I2CBus The I2C bus to use. Either Wire, Wire1, Wire2
 */
void initBMP(TwoWire* I2CBus);

/**
 * @brief Initializes the IMU sensor on an I2C bus
 * 
 * @param I2CBus The I2C bus to use. Either Wire, Wire1, Wire2
 */
void initIMU(TwoWire* I2CBus);