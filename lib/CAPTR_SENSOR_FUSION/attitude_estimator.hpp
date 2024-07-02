#ifndef ATT_HPP
#define ATT_HPP

#include <ArduinoEigenDense.h>
#include <ArduinoEigen/Eigen/Cholesky>
#include <Quaternion.h>

namespace UKF {
    #define X_DIM 7
    #define Z_DIM 3
    #define P_DIM 3
    #define BIAS {0, 0, 0}
    class Attitude {
        Eigen::VectorXd x_hat_;
        Eigen::VectorXd x_prior_;
        Eigen::MatrixXd P;

        Eigen::Vector3d z;
        Eigen::Vector3d z_prior_;
        Eigen::MatrixXd Q;


        Eigen::Vector3d omega;

        Attitude();

        Attitude(Quaternion starting_orientation);

        Attitude(Quaternion starting_orientation, Eigen::Vector3d starting_bias);

        void predict(double dt, Eigen::Vector3d w_m);

        void update(Eigen::VectorXd z_measurement);

        /**
         * @brief Dyanmic model that "rides the gyro" to predict the next state.
         * 
         * @param x The state vector. The first 4 elements are the quaternion, the next 3 are the bias.
         * @param w_measured The angular velocities. Assumed to be in the 3-2-1, or Z-Y-X convention.
         * @param dt The time step, in seconds.
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd f_quaternion(Eigen::VectorXd x, Eigen::Vector3d w_m, double dt);

        /**
         * @brief Measurement model. 
         * 
         * @param z 
         * @return Eigen::VectorXd 
         */
	    Eigen::VectorXd h_quaternion(Eigen::VectorXd z);
    } ;
}


#endif