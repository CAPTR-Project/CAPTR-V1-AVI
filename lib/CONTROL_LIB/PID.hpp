#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include "quaternion.h"  // custom quaternion library

class PID
{
public:
    PID(float dt, float max, float min, float Kp, float Kd, float Ki)        // has to be same name as class because it is a constructor like init in python
        : dt_(dt), max_(max), min_(min), Kp_(Kp), Kd_(Kd), Ki_(Ki), integral_(0.0), prev_error_(0.0) {}

    float run(float setpoint, float currentValue) {
        error_ = setpoint - currentValue;
        derivative_ = (error_ - prev_error_) / dt_;
        float res =  Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative_;

        if (res > max_) res = max_; 
        else if (res < min_) res = min_;
        else integral_ += error_ * dt_;

        prev_error_ = error_;
        return res;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    float Kp_, Ki_, Kd_;
    float min_, max_;
    float setpoint_;
    float error_, prev_error_, derivative_, integral_;
    double dt_;
};

#endif
