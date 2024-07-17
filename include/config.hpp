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
#define GYRO_X_BIAS 0.0 // [rad/s]
#define GYRO_Y_BIAS 0.0 // [rad/s]
#define GYRO_Z_BIAS 0.0 // [rad/s]

// Magnetometer
#define MAG_X_OFFSET 0.0 // [uT]
#define MAG_Y_OFFSET 0.0 // [uT]
#define MAG_Z_OFFSET 0.0 // [uT]


#endif