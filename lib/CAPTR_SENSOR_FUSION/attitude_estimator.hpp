#ifndef ATT_HPP
#define ATT_HPP

#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Cholesky>
#include <Quaternion.h>
#include "config.hpp"
#include <vector>

namespace UKF {
    #define CENTER_WEIGHT 1.0
    #define CENTER_WEIGHT_MEASUREMENT 1.0

    #define X_DIM 7
    #define Z_DIM 3
    #define P_DIM 3
    #define Q_DIM 3
    #define R_DIM 3
    #define BIAS {0, 0, 0}
    class Attitude {

    public:

        Eigen::VectorXd x_hat_;
        Eigen::VectorXd x_prior_;
        Eigen::MatrixXd P_;

        Eigen::Vector3d z_vec_;
        Eigen::Vector3d z_prior_;
        Eigen::MatrixXd Q_;

        Eigen::MatrixXd R_;

        Eigen::MatrixXd sigma_points;

        Eigen::Vector3d ang_vec; // [rad/s] angular velocity in [x, y, z] form. roll, pitch, yaw.

        Eigen::Vector3d mag_vec_up; // [uT] magnetic field vector in [x, y, z] form.

        std::atomic_bool initialized = false;

        std::atomic<double> quat_w;
        std::atomic<double> quat_x;
        std::atomic<double> quat_y;
        std::atomic<double> quat_z;

        /**
         * @brief Construct a new Attitude object. Initializes everything.
         * 
         */
        Attitude();

        Attitude    (Quaternion starting_orientation,
                    Eigen::Vector3d starting_bias, 
                    Eigen::Vector3d mag_vec, 
                    Eigen::Matrix<double, Q_DIM, Q_DIM> Q,
                    Eigen::Matrix<double, Z_DIM, Z_DIM> R);

        /**
         * @brief Predict the next state of the system. Uses the dynamic model to predict the next state.
         * 
         * @param dt [s] The time step.
         * @param w_m [rad/s] The gyroscope measurements. IMPORTANT: Assumed to be in the 3-2-1, or Z-Y-X convention.
         */
        void predict(double dt, Eigen::Vector3d w_m);

        /**
         * @brief 
         * 
         * @param z_measurement 
         */
        void update_mag(Eigen::Vector3d z_measurement);


        /**
         * @brief Dyanmic model that "rides the gyro" to predict the next state.
         * 
         * @param x [quat, rads] The state vector. The first 4 elements are the quaternion, the next 3 are the bias.
         * @param w_measured [rad] The angular velocities. Assumed to be in the 3-2-1, or Z-Y-X convention.
         * @param dt [2] The time step.
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd f_quaternion(Eigen::VectorXd x, Eigen::Vector3d w_m, double dt);

        /**
         * @brief Measurement model. 
         * 
         * @param z [uT] The magnetic field vector in the body frame. IMPORTANT: Must be corrected for hard and soft iron biases.
         * @return UnitQuaternion
         */
	    UnitQuaternion h_quaternion(Eigen::Vector3d z);
        
        
        void set_gyroBiases(Eigen::Vector3d new_biases);
        void set_magVec(Eigen::Vector3d new_magVec);
    } ;
}

#endif