#ifndef CFG_HPP
#define CFG_HPP

// ========================== General configuration ===========================

#define LOGGING_FREQUENCY 50 // [Hz] The frequency at which to log data.
#define CONTROL_FREQUENCY 200 // [Hz] The frequency at which to run the control loop.


// ====================== State estimation configuration ======================

#define Q_MATRIX (Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity() * 0.1) // The process noise matrix.
#define R_MATRIX (Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity() * 0.1) // The measurement noise matrix.



// ============================ Sensor configuration ==========================

// Gyroscope
#define IMU_DATARATE LSM6DS_RATE_104_HZ
#define GYRO_CALIBRATION_TIME 10000 // [ms] The time to calibrate the gyroscope offsets.
#define GYRO_INT_PIN 2 // The pin to which the gyroscope interrupt is connected.

// Accelerometer
#define ACCEL_INT_PIN 3 // The pin to which the accelerometer interrupt is connected.

// Magnetometer
#define MAG_X_OFFSET 0.0 // [uT]
#define MAG_Y_OFFSET 0.0 // [uT]
#define MAG_Z_OFFSET 0.0 // [uT]
#define MAG_DATARATE LIS3MDL_DATARATE_40_HZ
#define MAG_INT_PIN 4 // The pin to which the magnetometer interrupt is connected.

// Barometer
#define BARO_INT_PIN 5 // The pin to which the barometer interrupt is connected.
#define BARO_PRESSURE_ASL 1013.25 // [Pa] The pressure at sea level.


// Flash configuration
#define FLASH_CHIP 1 // Chip select pin for flash memory CHANGE WHEN IMPLEMENTED!!!!!

#endif