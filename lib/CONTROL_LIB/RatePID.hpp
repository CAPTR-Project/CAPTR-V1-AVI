#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include "PID.hpp"  // custom PID library
#include "quaternion.h"  // custom quaternion library

class RatePID
{
public:
    RatePID(double dt, Eigen::Vector3f max, Eigen::Vector3f min, Eigen::Vector3f Kp, Eigen::Vector3f Kd, Eigen::Vector3f Ki)
    : 

    // PID functions
    Eigen::Vector3f compute(Eigen::Vector3f rateSetpoint, Eigen::Vector3f currentRate){
        
    }
    void reset();

private:
    Eigen::Vector3f min_, max_;
    PID pid_;
};

#endif
