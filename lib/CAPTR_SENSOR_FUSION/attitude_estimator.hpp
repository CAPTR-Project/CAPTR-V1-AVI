#ifndef ATT_HPP
#define ATT_HPP

#include <ArduinoEigenDense.h>
#include <quaternion.h>

namespace UKF {
    class Attitude {
        int x_dim_;
        Eigen::VectorXd x_hat_;
        Eigen::VectorXd x_prior_;
        Eigen::MatrixXd P;

        int z_dim_;
        Eigen::VectorXd z;
        Eigen::VectorXd z_prior_;

        Eigen::MatrixXd omega;

        Attitude();

        void predict(double dt);

        void update(Eigen::VectorXd z_measurement);

        /**
         * @brief Dyanmic model that "rides the gyro" to predict the next state.
         * 
         * @param x The state vector. The first 4 elements are the quaternion, the next 3 are the bias.
         * @param w_measured The angular velocities. Assumed to be in the 3-2-1, or Z-Y-X convention.
         * @param dt The time step, in seconds.
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd f_quaternion(Eigen::VectorXd x, Eigen::VectorXd w_measured, double dt);

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