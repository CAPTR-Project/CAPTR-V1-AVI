#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include "quaternion.h"  // custom quaternion library

class PID
{
public:
    PID(float dt, float max, float min, float Kp, float Ki, float Kd, float integ_clamp, float alpha, float tau)        // has to be same name as class because it is a constructor like init in python
        : dt_(dt), max_(max), min_(min), Kp_(Kp), Ki_(Ki), Kd_(Kd), integ_clamp_(integ_clamp), alpha_(alpha), alpha_lpf(dt / (dt + tau)), 
            integral_(0.0), prev_error_(0.0), old_error_filtered_(0.0), old_output_filtered_(0.0) {}

    float run(float setpoint, float currentValue) {
        error_ = setpoint - currentValue;
        ef_ = alpha_ * error_ + (1 - alpha_) * old_error_filtered_;

        derivative_ = alpha_ * (error_ - prev_error_) + (1 - alpha_) * derivative_;
        output_ =  Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative_;

        if (output_ > max_) output_ = max_; 
        else if (output_ < min_) output_ = min_;
        else if (abs(integral_) < integ_clamp_) integral_ += error_ * dt_;

        output_ = alpha_lpf * output_ + (1 - alpha_lpf) * old_output_filtered_; // Low pass filter

        // Update old values for next iteration
        prev_error_ = error_;
        old_error_filtered_ = ef_;
        old_output_filtered_ = output_;

        return output_;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    double dt_;
    float max_, min_;
    float Kp_, Ki_, Kd_, integ_clamp_;
    float error_, derivative_, alpha_, alpha_lpf;
    float integral_, prev_error_, old_error_filtered_, old_output_filtered_;
    float ef_, output_;

};

#endif
