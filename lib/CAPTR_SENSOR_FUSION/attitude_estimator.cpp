#include <attitude_estimator.hpp>

namespace UKF {

Attitude::Attitude(){
    x_hat_ = Eigen::VectorXd(X_DIM);
    x_hat_ << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd bias BIAS;
    x_hat_.block<3, 1>(4, 0) = bias;
    x_prior_ = Eigen::VectorXd(X_DIM);

    P = Eigen::MatrixXd(P_DIM, P_DIM);

    z = Eigen::VectorXd(Z_DIM);
    z_prior_ = Eigen::VectorXd(Z_DIM);

    Q = Eigen::MatrixXd(Z_DIM, Z_DIM);

    omega = Eigen::Vector3d(0, 0, 0);
}

Attitude::Attitude(Quaternion starting_orientation, Eigen::Vector3d starting_bias) {
    x_hat_ = Eigen::VectorXd(X_DIM);
    x_hat_.block<4, 1>(0, 0) = starting_orientation.to_quaternion_vector();

    x_hat_.block<3, 1>(4, 0) = starting_bias;

    x_prior_ = x_hat_;

    P = Eigen::MatrixXd(P_DIM, P_DIM);

    z = Eigen::VectorXd(Z_DIM);
    z_prior_ = Eigen::VectorXd(Z_DIM);

    Q = Eigen::MatrixXd(Z_DIM, Z_DIM);

    omega = Eigen::Vector3d(0, 0, 0);
}

void Attitude::predict(double dt, Eigen::Vector3d w_m) {
    x_prior_ = Eigen::VectorXd(x_hat_);
    // generate sigma points
    Eigen::MatrixXd sigma_points(P_DIM, 2 * P_DIM + 1);

    Eigen::MatrixXd covSqrt = P.ldlt().matrixL();

    sigma_points.block<P_DIM, 1>(0, 0) = x_hat_;
    for (int i = 1; i <= P_DIM; i++) {
        sigma_points.block<P_DIM, 1>(0, i) = sqrt(2 * (i)) * covSqrt.col(i);
        sigma_points.block<P_DIM, 1>(0, i + P_DIM) = -sqrt(2 * (i)) * covSqrt.col(i);
    }

    Eigen::MatrixXd quat_sigma_points(X_DIM, 2 * P_DIM + 1);

    UnitQuaternion q_k(x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3));
    UnitQuaternion q_noise;
    // Eigen::Vector3d w_noise;
    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        q_noise = UnitQuaternion::omega(sigma_points(0, i), sigma_points(1, i), sigma_points(2, i));
        // w_noise = sigma_points.block<3, 1>(4, i);

        quat_sigma_points.block<4, 1>(0, i) = (q_k * q_noise).to_quaternion_vector();

        quat_sigma_points.block<3, 1>(4, i) = x_hat_.block<3, 1>(4, 0);
    }

    // predict sigma points

    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        quat_sigma_points.block<4, 1>(0, i) = f_quaternion(quat_sigma_points.block<X_DIM, 1>(0, i), w_m, dt);
    }
    
    // predict mean and covariance
    UnitQuaternion est_mean = UnitQuaternion(x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3));
    for (int i = 0; i < 1000; i++) {
        
    }
    
}

Eigen::VectorXd Attitude::f_quaternion(Eigen::VectorXd x, Eigen::Vector3d w_m, double dt) {
    UnitQuaternion q_k(x(0), x(1), x(2), x(3));

    Eigen::Vector3d bias(3);
    bias << x(4), x(5), x(6);

    omega = w_m - bias;

    UnitQuaternion dq = UnitQuaternion::omega(dt * omega(0), dt * omega(1), dt * omega(2));

    q_k = q_k * dq;

    Eigen::VectorXd x_k_plus_1(X_DIM);

    x_k_plus_1.block<4, 1>(0, 0) = q_k.to_quaternion_vector();

    x_k_plus_1.block<3, 1>(4, 0) = bias;
    
    return x_k_plus_1;
}


}