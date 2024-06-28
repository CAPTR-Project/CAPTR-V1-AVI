#include <attitude_estimator.hpp>

namespace UKF {

Attitude::Attitude(){
    x_dim_ = 10;
    x_hat_ = Eigen::VectorXd(x_dim_);
    x_hat_ << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    x_prior_ = Eigen::VectorXd(x_dim_);
    P = Eigen::MatrixXd(x_dim_, x_dim_);

    z_dim_ = 6;
    z = Eigen::VectorXd(z_dim_);
    z_prior_ = Eigen::VectorXd(z_dim_);

    omega = Eigen::MatrixXd(4, 4);

}

void Attitude::predict(double dt) {

}

// Eigen::VectorXd Attitude::f_quaternion(Eigen::VectorXd x, Eigen::VectorXd w_measured, double dt) {
//     Eigen::VectorXd q_k(x(0), x(1), x(2), x(3));
//     Eigen::VectorXd bias(x(4), x(5), x(6));

//     Eigen::VectorXd w_k(w_measured(0),
//                         w_measured(1),
//                         w_measured(2));

//     w_k -= bias;


//     Eigen::Vector3d psci(3);

//     psci = (sin(0.5 * dt * w_k.norm()) * w_k) / w_k.norm();
//     Eigen::Matrix2d psci_x = {0, -psci(2), psci(1),
//                               psci(2), 0, -psci(0),
//                               -psci(1), psci(0), 0};

//     omega.block<3, 3>(0, 0) = (cos(0.5 * dt * w_k.norm()) * Eigen::MatrixXd::Ones(3, 3)) - psci_x;
//     omega.block<3, 1>(0, 3) = psci;
//     omega.block<1, 3>(3, 0) = -1 * psci.transpose();
//     omega(3, 3) = cos(0.5 * dt * w_k.norm());

//     Eigen::VectorXd x_k_plus_1(x_dim_);

//     x_k_plus_1.block<4, 1>(0, 0) = omega * q_k;

//     x_k_plus_1.block<3, 1>(4, 0) = bias;
    
//     return x_k_plus_1;
// }

Eigen::VectorXd Attitude::f_quaternion(Eigen::VectorXd x, Eigen::VectorXd w_measured, double dt) {
    quaternion::Quaternion q_k(x(0), x(1), x(2), x(3));

    Eigen::Vector3d bias(3);
    bias << x(4), x(5), x(6);

    Eigen::Vector3d w_k = w_measured - bias;

    Eigen::VectorXd dw = 0.5 * dt * w_k;
    std::array<double, 3> dw_arr = {dw(0), dw(1), dw(2)};

    quaternion::Quaternion dq = quaternion::from_euler(dw_arr);

    quaternion::Quaternion q_k_plus_1 = dq * q_k;

    Eigen::VectorXd x_k_plus_1(x_dim_);

    x_k_plus_1.block<4, 1>(0, 0) = q_k_plus_1.to_eigen();

    x_k_plus_1.block<3, 1>(4, 0) = bias;
    
    return x_k_plus_1;
}


}