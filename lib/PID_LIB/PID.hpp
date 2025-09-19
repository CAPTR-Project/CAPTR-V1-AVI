/**
 * @file PID.hpp
 * @brief Standard form PID controller implementation (derivative on measurement) with integral clamping and output filtering
 * @author Tommy Zhang
 * 
 * Copyright (c) 2025 Tommy Zhang
 * SPDX-License-Identifier: MIT
 * 
 * This file is part of cpp-pid
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PID_SF_HPP
#define PID_SF_HPP

#include <iostream>

class PID
{
public:
    /**
     * @brief PID controller constructor
     * @param Ts Time step for the controller
     * @param max Maximum output value
     * @param min Minimum output value
     * @param K System gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param integ_clamp Integral clamp value normalized to effector output (e.g. 500 for 500us on a servo)
     * @param alpha_d Smoothing factor for the derivative term (larger alpha_d means less smoothing)
     * @param tau Time constant for low-pass filter on output (larger tau is more smoothing)
     */
    PID(float Ts, float max, float min, float K, float Kp, float Ki, float Kd, float integ_clamp, float alpha_d, float tau_output)
        : Ts_(Ts), max_(max), min_(min), derivative_(0.0), alpha_d_(alpha_d),
          alpha_lpf((Ts + tau_output) > 1e-6f ? Ts / (Ts + tau_output) : 1.0f)
    {
        if (Ts_ == 0.0f) Ts_ = 1e-6f; // prevent division by zero

        setK(K);
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
        setIntegClamp(integ_clamp);
    }

    float run(float setpoint, float measurement) {
        error_ = setpoint - measurement;

        if (Ts_ > 0.0f) derivative_ = alpha_d_ * (measurement - prev_meas_) + (1 - alpha_d_) * derivative_;
        output_ =  k_ * (kp_ * error_ + ki_Ts_ * integral_ - kd_Ts_ * derivative_);

        if (output_ > max_) output_ = max_; 
        else if (output_ < min_) output_ = min_;
        else integral_ += error_;
        
        // integral clamp
        if (integral_ > integ_clamp_) integral_ = integ_clamp_;
        else if (integral_ < -integ_clamp_) integral_ = -integ_clamp_;

        output_ = alpha_lpf * output_ + (1 - alpha_lpf) * old_output_filtered_; // increasing tau makes output slower

        // Update old values for next iteration
        prev_meas_ = measurement;
        old_output_filtered_ = output_;

        return output_;
    }

    void reset() {
        integral_ = 0.0;
        prev_meas_ = 0.0;
    }
    float integral_;

private:
    double Ts_ = 0.1;
    float max_, min_ = 0.0f;
    float k_, kp_, ki_Ts_, kd_Ts_, integ_clamp_ = 0.0f;
    float error_, derivative_, alpha_d_, alpha_lpf = 0.0f;
    float prev_meas_, old_error_filtered_, old_output_filtered_ = 0.0;
    float output_;

    void setK(float K) { this->k_ = K; }
    void setKp(float Kp) { this->kp_ = Kp; }  // for compatibility with other PID libraries
    void setKi(float Ki) { this->ki_Ts_ = Ki * Ts_; }  // premultiply by Ts for efficiency
    void setKd(float Kd) { this->kd_Ts_ = Kd / Ts_; }  // predivide by Ts for efficiency
    void setIntegClamp(float integ_clamp) { this->integ_clamp_ = (k_ * ki_Ts_ > 0) ? integ_clamp / (k_ * ki_Ts_) : 0.0f; }
};

#endif