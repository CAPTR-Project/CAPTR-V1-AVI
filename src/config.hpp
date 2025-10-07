#ifndef CONFIG_HPP
#define CONFIG_HPP

// ========================== General configuration ===========================

#define LOGGING_FREQUENCY 50 // [Hz] The frequency at which to log data.
#define CONTROL_FREQUENCY 100 // [Hz] The frequency at which to run the control loop.
#define FSM_FREQUENCY 30 // [Hz] The frequency at which to run the finite state machine.

#define RADIO_ROCKET_ADDRESS 1 // The address of the radio client. ENSURE SAME AS GROUND STATION
#define RADIO_GS_ADDRESS 2 // The address of the radio server. ENSURE SAME AS GROUND STATION
#define RF95_FREQ 915.0 // [MHz] The frequency of the radio module.
#define RFM95_CS 10 // Chip select pin for the radio module
#define RFM95_INT 2 // Interrupt pin for the radio module

// ====================== State estimation configuration ======================

#define Q_MATRIX (Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity() * 0) // The process noise matrix.
#define R_MATRIX (Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity() * 0.01) // The measurement noise matrix.

// ============================ Sensor configuration ==========================

// Gyroscope
#define ACCEL_DATARATE LSM6DS_RATE_52_HZ
#define GYRO_DATARATE LSM6DS_RATE_416_HZ
#define GYRO_CALIBRATION_TIME 6000 // [ms] The time to calibrate the gyroscope offsets.
#define GYRO_INT_PIN 40 // The pin to which the gyroscope interrupt is connected.

// Accelerometer
#define ACCEL_INT_PIN 39 // The pin to which the accelerometer interrupt is connected.
#define ORIENTATION_CALIBRATION_TIME 8000 // [ms] The time to calibrate the accelerometer offsets.

// Magnetometer
#define MAG_X_OFFSET 0.0 // [uT]
#define MAG_Y_OFFSET 0.0 // [uT]
#define MAG_Z_OFFSET 0.0 // [uT]
#define MAG_DATARATE LIS3MDL_DATARATE_10_HZ
#define MAG_INT_PIN 38 // The pin to which the magnetometer interrupt is connected.

// Barometer
#define BARO_INT_PIN 41 // The pin to which the barometer interrupt is connected.
#define BARO_PRESSURE_ASL 101325 // [Pa] The pressure at sea level.
#define BARO_DATARATE BMP3_ODR_200_HZ

// GPS
#define GPS_SERIAL_PORT Serial2 // The serial port to which the GPS is connected.
#define GPS_BAUDRATE 9600 // The baudrate of the GPS.
#define GPS_FREQUENCY 100 // [Hz] The frequency at which to read the GPS.

// Flash configuration
#define FLASH_CHIP 10 // Chip select pin for flash memory

// TVC mount configuration
#define SERVO_PITCH_PIN 22
#define SCALING_PITCH 1.3
#define OFFSET_PITCH 0.0
#define LIMIT_PITCH 0.16
#define SERVO_YAW_PIN 23
#define SCALING_YAW 1.3
#define OFFSET_YAW 0.0
#define LIMIT_YAW 0.16

#endif // CONFIG_HPP