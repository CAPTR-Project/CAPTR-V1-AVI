#include "captr_sensor_msgs.hpp"

namespace sensor_msgs {
    // GyroMsg
    GyroMsg::GyroMsg() {
        ready = xSemaphoreCreateMutex();
    }
    Eigen::Vector3d GyroMsg::toVector() {
        return Eigen::Vector3d(z, y, x);
    }
    Eigen::Vector3d GyroMsg::getBiasVector() {
        return Eigen::Vector3d(z_bias, y_bias, x_bias);
    }

    // AccelMsg
    AccelMsg::AccelMsg() {
        ready = xSemaphoreCreateMutex();
    }
    Eigen::Vector3d AccelMsg::toVector() {
        return Eigen::Vector3d(x, y, z);
    }

    // MagMsg
    MagMsg::MagMsg() {
        ready = xSemaphoreCreateMutex();
    }
    Eigen::Vector3d MagMsg::toVector() {
        return Eigen::Vector3d(x, y, z);
    }
    // BaroMsg
    BaroMsg::BaroMsg() {
        ready = xSemaphoreCreateMutex();
    }
}