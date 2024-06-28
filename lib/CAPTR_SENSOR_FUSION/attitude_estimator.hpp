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


        Eigen::VectorXd f_quaternion(Eigen::VectorXd x, Eigen::VectorXd w_k_minus_1, double dt);
	    Eigen::VectorXd h_quaternion(Eigen::VectorXd x);
    } ;
}


#endif