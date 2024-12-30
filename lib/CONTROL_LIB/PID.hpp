#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include "quaternion.h"  // custom quaternion library

class PID
{
public:
    PID(float dt, float max, float min, float Kp, float Ki, float Kd, float N)        // has to be same name as class because it is a constructor like init in python
        : dt_(dt), max_(max), min_(min), Kp_(Kp), Ki_(Ki), Kd_(Kd), N_(N), integral_(0.0), prev_error_(0.0) {}

    float run(float setpoint, float currentValue) {
        error_ = setpoint - currentValue;
        derivative_ = N_ * (error_ - prev_error_) / (1 + N_ * dt_); // filtered derivative: formula: N/(1+NT_s) * (e(t) - e(t-1))
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
    double dt_;
    float max_, min_;
    float Kp_, Ki_, Kd_;
    float error_, derivative_, integral_, prev_error_;
    float N_;

};

#endif
