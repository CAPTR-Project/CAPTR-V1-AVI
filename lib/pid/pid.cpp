#include "PID.hpp"

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki, double intMax): 
    _dt(dt),
    _max(max), 
    _min(min), 
    _Kp(Kp), 
    _Kd(Kd), 
    _Ki(Ki), 
    _integral(0.0), 
    _prevError(0.0),
    _integral_max(intMax)
{
}

void PID::setDt(double dt)
{
    _dt = dt;
}

void PID::setSetpoint(double setpoint)
{
    _setpoint = setpoint;
}

double PID::update(double input)
{
    _error = _setpoint - input;                                   // P
    _integral += _error * _dt;                                           // I
    _derivative = (_error - _prevError) / _dt;                     // D

    if ((_error / _prevError) < 0){             // integral windup prevention - if error has overshot, reset integral
        reset(false);
    }

    // sum
    _prevError = _error;
    double output = _Kp * _error + _Ki * _integral + _Kd * _derivative;

    // clamping to min and max
    if (output < _min)
    {
        output = _min;
    }
    else if (output > _max)
    {
        output = _max;
    }
    return output;
}

void PID::reset(bool all)
{
    if (all)
    {
        _integral = 0.0;
        _prevError = 0.0;
    }
    else
    {
        _integral = 0.0;
    }
    
}


PID::~PID()
{
}