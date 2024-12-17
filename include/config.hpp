#ifndef CFG_HPP
#define CFG_HPP

// ========================== General configuration ===========================

#define LOGGING_FREQUENCY 10 // [Hz] The frequency at which to log data.
#define CONTROL_FREQUENCY 100 // [Hz] The frequency at which to run the control loop.
#define FSM_FREQUENCY 10 // [Hz] The frequency at which to run the finite state machine.

// ====================== State estimation configuration ======================

#define Q_MATRIX (Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity() * 0) // The process noise matrix.
#define R_MATRIX (Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity() * 0.01) // The measurement noise matrix.



// ============================ Sensor configuration ==========================

// Gyroscope
#define IMU_DATARATE LSM6DS_RATE_208_HZ
#define GYRO_CALIBRATION_TIME 5000 // [ms] The time to calibrate the gyroscope offsets.
#define GYRO_INT_PIN 40 // The pin to which the gyroscope interrupt is connected.

// Accelerometer
#define ACCEL_INT_PIN 39 // The pin to which the accelerometer interrupt is connected.

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
#define GPS_FREQUENCY 10 // [Hz] The frequency at which to read the GPS.

// Flash configuration
#define FLASH_CHIP 10 // Chip select pin for flash memory

// TVC mount configuration
#define SERVO_PITCH_PIN 14
#define SERVO_YAW_PIN 15

#endif