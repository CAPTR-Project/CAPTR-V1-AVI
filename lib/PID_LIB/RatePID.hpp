#ifndef RATEPID_HPP
#define RATEPID_HPP

#include <iostream>
#include "PID.hpp"  // custom PID library
#include "quaternion.h"  // custom quaternion library

class RatePID
{
public:
    RatePID(float dt, float max, float min, Eigen::Vector3d K, Eigen::Vector3d Kp, Eigen::Vector3d Ki, Eigen::Vector3d Kd, Eigen::Vector3d integ_clamp, float alpha, float tau)
    : pidX_(dt, max, min, K(0), Kp(0), Ki(0), Kd(0), integ_clamp(0), alpha, tau),
    pidY_(dt, max, min, K(1), Kp(1), Ki(1), Kd(1), integ_clamp(1), alpha, tau),
    pidZ_(dt, max, min, K(2), Kp(2), Ki(2), Kd(2), integ_clamp(2), alpha, tau) {}

    // PID functions
    Eigen::Vector3d compute(Eigen::Vector3d rateSetpoint, Eigen::Vector3d currentRate) {
        // return directly as Eigen::Vector3d
        return {
            pidZ_.run(rateSetpoint(0), currentRate(0)),
            pidY_.run(rateSetpoint(1), currentRate(1)),
            pidX_.run(rateSetpoint(2), currentRate(2))
        };
    }
    void reset(){
        pidX_.reset();
        pidY_.reset();
        pidZ_.reset();
    }

    PID pidX_;
    PID pidY_;
    PID pidZ_;
};

#endif
