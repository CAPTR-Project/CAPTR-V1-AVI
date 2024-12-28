#ifndef RATEPID_HPP
#define RATEPID_HPP

#include <iostream>
#include "PID.hpp"  // custom PID library
#include "quaternion.h"  // custom quaternion library

class RatePID
{
public:
    RatePID(float dt, float max, float min, Eigen::Vector3f Kp, Eigen::Vector3f Ki, Eigen::Vector3f Kd)
    : pidX_(dt, max, min, Kp(0), Ki(0), Kd(0)),
    pidY_(dt, max, min, Kp(1), Ki(1), Kd(1)),
    pidZ_(dt, max, min, Kp(2), Ki(2), Kd(2)) {}

    // PID functions
    Eigen::Vector3f compute(Eigen::Vector3f rateSetpoint, Eigen::Vector3f currentRate) {
        // return directly as Eigen::Vector3f
        return {
            pidX_.run(rateSetpoint(0), currentRate(0)),
            pidY_.run(rateSetpoint(1), currentRate(1)),
            pidZ_.run(rateSetpoint(2), currentRate(2))
        };
    }
    void reset(){
        pidX_.reset();
        pidY_.reset();
        pidZ_.reset();
    }

private:
    PID pidX_;
    PID pidY_;
    PID pidZ_;
};

#endif
