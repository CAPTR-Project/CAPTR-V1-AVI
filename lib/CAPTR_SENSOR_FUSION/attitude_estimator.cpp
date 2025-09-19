#include <attitude_estimator.hpp>

namespace UKF {

Attitude::Attitude(){
    ready = xSemaphoreCreateMutex();
    
    // Initialize fixed-size matrices to zero
    sigma_points.setZero();
    quat_sigma_points.setZero();
    z_sigma_points.setZero();
    
    // init(UnitQuaternion(1, 0, 0, 0),
    // Eigen::Vector3d(0, 0, 0),
    // Eigen::Vector3d(0, 0, 0),
    // Eigen::Matrix<double, Q_DIM, Q_DIM>::Identity(),
    // Eigen::Matrix<double, Z_DIM, Z_DIM>::Identity());
    initialized = false;
}

void Attitude::init(UnitQuaternion starting_orientation, 
                    Eigen::Vector3d starting_bias, 
                    Eigen::Vector3d mag_vec, 
                    Eigen::Matrix<double, Q_DIM, Q_DIM> Q,
                    Eigen::Matrix<double, Z_DIM, Z_DIM> R) 
{
    x_hat_ = Eigen::VectorXd::Zero(X_DIM);
    x_hat_.block<4, 1>(0, 0) = starting_orientation.to_quaternion_vector();
    x_hat_.block<3, 1>(4, 0) = starting_bias;
    mag_vec_up = mag_vec;

    q_k_ = starting_orientation;

    x_prior_ = x_hat_;

    P_ = Eigen::Matrix3d::Zero(P_DIM, P_DIM);

    z_vec_ = Eigen::VectorXd(Z_DIM);
    z_prior_ = Eigen::VectorXd(Z_DIM);

    Q_ = Q;

    R_ = R;

    // sigma_points and quat_sigma_points are now pre-allocated in constructor
    // sigma_points = Eigen::MatrixXd::Zero(P_DIM, 2 * P_DIM + 1);

    ang_vec = Eigen::Vector3d(0, 0, 0);

    // quat_sigma_points = Eigen::MatrixXd(X_DIM, 2 * P_DIM + 1);

    if (xSemaphoreTake(ready, 1) == pdTRUE) {
        newest_attitude_quat = starting_orientation;
        xSemaphoreGive(ready);
        initialized = true;
    }

}

void Attitude::predict_integrate(double dt, Eigen::Vector3d w_m) {
    x_hat_ = f_quaternion(x_hat_, w_m, dt);
    q_k_ = UnitQuaternion(x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3));
    if (xSemaphoreTake(ready, 1) == pdTRUE) {
        newest_attitude_quat = q_k_;
        xSemaphoreGive(ready);
    }
}

void Attitude::predict(double dt, Eigen::Vector3d w_m) {
    x_prior_ = Eigen::VectorXd(x_hat_);
    // generate sigma points
    // cholesky factorize P_
    covSqrt = (P_ + Q_).llt().matrixU();

    // Serial.printf("P_ + Q_: %.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n", (P_ + Q_)(0, 0), (P_ + Q_)(0, 1), (P_ + Q_)(0, 2), (P_ + Q_)(1, 0), (P_ + Q_)(1, 1), (P_ + Q_)(1, 2), (P_ + Q_)(2, 0), (P_ + Q_)(2, 1), (P_ + Q_)(2, 2));


    // Serial.printf("%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n%.2f, %.2f, %.2f\n", covSqrt(0, 0), covSqrt(0, 1), covSqrt(0, 2), covSqrt(1, 0), covSqrt(1, 1), covSqrt(1, 2), covSqrt(2, 0), covSqrt(2, 1), covSqrt(2, 2));

    // q_k.s = x_hat_(0); q_k.v_1 = x_hat_(1); q_k.v_2 = x_hat_(2); q_k.v_3 = x_hat_(3);

    sigma_points.block<P_DIM, 1>(0, 0) = Eigen::Vector3d(0, 0, 0);
    for (int i = 1; i <= P_DIM; i++) {
        sigma_points.block<P_DIM, 1>(0, i) = sqrt(2 * P_DIM) * covSqrt.col(i - 1);
        sigma_points.block<P_DIM, 1>(0, i + P_DIM) = -sqrt(2 * P_DIM) * covSqrt.col(i - 1);
    }

    UnitQuaternion q_noise;
    // Eigen::Vector3d w_noise;
    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        q_noise = UnitQuaternion::from_rotVec(sigma_points(0, i), sigma_points(1, i), sigma_points(2, i));
        // w_noise = sigma_points.block<3, 1>(4, i);

        quat_sigma_points.block<4, 1>(0, i) = (q_noise * q_k_).to_quaternion_vector();

        quat_sigma_points.block<3, 1>(4, i) = x_hat_.block<3, 1>(4, 0);
    }

    // predict sigma points

    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        quat_sigma_points.block<X_DIM, 1>(0, i) = f_quaternion(quat_sigma_points.block<X_DIM, 1>(0, i), w_m, dt);
    }
    
    // predict mean and covariance
    UnitQuaternion est_mean = q_k_;
    Eigen::Vector3d euler = est_mean.to_euler();

    // Serial.printf("%.2f, %.2f, %.2f\n", euler(0), euler(1), euler(2));
    // Serial.printf("%.2f, %.2f, %.2f, %.2f\n", est_mean.s, est_mean.v_1, est_mean.v_2, est_mean.v_3);

    Eigen::Vector3d avg_err;

    for (int i = 0; i < 50; i++) {
        avg_err = Eigen::Vector3d::Zero();
        for (int j = 0; j < 2 * P_DIM + 1; j++) {
            UnitQuaternion q_j(quat_sigma_points(0, j), quat_sigma_points(1, j), quat_sigma_points(2, j), quat_sigma_points(3, j));
            q_j = (q_j * est_mean.inverse());
            if (j == 0) avg_err += CENTER_WEIGHT * q_j.to_rotVec();
            else avg_err += q_j.to_rotVec();
        }
        avg_err /= (2 * P_DIM + CENTER_WEIGHT);

        est_mean = UnitQuaternion::from_rotVec(avg_err(0), avg_err(1), avg_err(2)) * est_mean;
    }
    // Serial.printf("estimated: %.8f, %.8f, %.8f, %.8f\n", est_mean.s, est_mean.v_1, est_mean.v_2, est_mean.v_3);


    q_k_ = est_mean;

    x_prior_ = x_hat_;

    x_hat_.block<4, 1>(0, 0) = q_k_.to_quaternion_vector();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        UnitQuaternion q_j(quat_sigma_points(0, i), quat_sigma_points(1, i), quat_sigma_points(2, i), quat_sigma_points(3, i));
        q_j = (q_j * q_k_.inverse());
        double theta = 2 * acos(q_j.s);
        Eigen::Vector3d err = Eigen::Vector3d::Zero();
        if (abs(theta) > 1e-12) err = theta / sin(theta) * q_j.to_quaternion_vector().block<3, 1>(1, 0);
        sigma_points.block<3, 1>(0, i) = err;
        cov += err * err.transpose();
    }
    cov /= (2 * P_DIM + 1);

    P_ = cov;

    ang_vec = w_m - bias;

    if (xSemaphoreTake(ready, pdMS_TO_TICKS(3)) == pdTRUE) {
        newest_attitude_quat = q_k_;
        xSemaphoreGive(ready);
    }
    
}

void Attitude::update_mag(Eigen::Vector3d measurement) {
    UnitQuaternion x_k_quat = UnitQuaternion(x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3));

    for (int i = 0; i < 2 * R_DIM + 1; i++) {
        UnitQuaternion q_j(sigma_points(0, i), sigma_points(1, i), sigma_points(2, i), sigma_points(3, i));
        z_sigma_points.block<R_DIM, 1>(0, i) = q_j.vector_rotation_by_quaternion(mag_vec_up);
    }

    // calculate uncertainty in measurement caused by uncertainty in state, P_zz
    Eigen::Vector3d z_k_predicted = x_k_quat.vector_rotation_by_quaternion(mag_vec_up);

    Eigen::Matrix3d P_zz = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 2 * R_DIM + 1; i++) {
        P_zz += (z_sigma_points.block<R_DIM, 1>(0, i) - z_k_predicted) * (z_sigma_points.block<R_DIM, 1>(0, i) - z_k_predicted).transpose();
    }
    P_zz /= (2 * R_DIM + 1);

    // calculate cross covariance matrix
    Eigen::MatrixXd P_xz = Eigen::MatrixXd::Zero(P_DIM, R_DIM);

    for (int i = 0; i < 2 * P_DIM + 1; i++) {
        P_xz += sigma_points.block<P_DIM, 1>(0, i) * (z_sigma_points.block<R_DIM, 1>(0, i) - z_k_predicted).transpose();
    }
    P_xz /= (2 * P_DIM + 1);

    // calculate kalman gain

    Eigen::MatrixXd K = P_xz * (P_zz + R_).inverse();

    // calculate weighted average of xhat and z to get new xhat. 
    Eigen::Vector3d innovation_rotvec = K * h_quaternion(measurement, z_k_predicted).to_rotVec();

    UnitQuaternion innovation = UnitQuaternion::from_rotVec(innovation_rotvec(0), innovation_rotvec(1), innovation_rotvec(2));

    UnitQuaternion new_orientation = x_k_quat * innovation;

    x_prior_ = x_hat_;

    x_hat_.block<4, 1>(0, 0) = new_orientation.to_quaternion_vector();

    if (xSemaphoreTake(ready, pdMS_TO_TICKS(3)) == pdTRUE) {
        newest_attitude_quat = new_orientation;
        xSemaphoreGive(ready);
    }
    
}

Eigen::VectorXd Attitude::f_quaternion(Eigen::VectorXd x, Eigen::Vector3d w_m, double dt) {
    UnitQuaternion q_k(x(0), x(1), x(2), x(3));

    ang_vec = w_m - bias;

    UnitQuaternion dq = UnitQuaternion::from_euler(dt * ang_vec(0), dt * ang_vec(1), dt * ang_vec(2));

    q_k = q_k * dq;

    q_k.normalize();

    // Eigen::Vector3d euler = q_k.to_euler();

    // Serial.printf("%.8f, %.8f, %.8f\n", euler(0), euler(1), euler(2));

    Eigen::VectorXd x_k_plus_1 = Eigen::VectorXd::Zero(X_DIM);

    x_k_plus_1.block<4, 1>(0, 0) = q_k.to_quaternion_vector();

    x_k_plus_1.block<3, 1>(4, 0) = bias;
    
    return x_k_plus_1;
}

UnitQuaternion Attitude::h_quaternion(Eigen::Vector3d z, Eigen::Vector3d z_pred) {
    // calculate rotation needed to take mag_vec_up to z
    Eigen::Vector3d axis = z_pred.cross(z);
    double angle = acos(z_pred.dot(z) / (z_pred.norm() * z.norm()));

    return UnitQuaternion::from_rotVec(angle * axis(0), angle * axis(1), angle * axis(2));
}

void Attitude::set_gyroBiases(float z, float y, float x) {
    bias = Eigen::Vector3d(z, y, x);
    x_hat_.block<3, 1>(4, 0) = bias;
}

void Attitude::set_magVec(float x, float y, float z) {
    mag_vec_up = Eigen::Vector3d(x, y, z);
}

} // namespace UKF