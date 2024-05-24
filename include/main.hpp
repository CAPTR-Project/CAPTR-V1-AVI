#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <TinyGPS++.h>

//Definitions
#define RFM95_CS    4  // TODO CHANGE
#define RFM95_INT   3  // TODO CHANGE
#define RFM95_RST   2  // TODO CHANGE

// Who am i? (server address)
#define SELF_RADIO_ADDRESS  1

// Where to send packets to! MY_ADDRESS in client (RX) should match this.
#define DEST_RADIO_ADDRESS  2

// Variables
Adafruit_BMP3XX bmp; // barometer
Adafruit_LSM6DS3TRC lsm; // IMU
Adafruit_Sensor *lsm_accel, *lsm_gyro;
sensors_event_t accelMain, gyro;
// radio
RH_RF95 rf95_driver(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95_driver, SELF_RADIO_ADDRESS);

// GPS
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

/**
 * @brief Initializes the RFM95 radio
 * 
 */
void initRadio();