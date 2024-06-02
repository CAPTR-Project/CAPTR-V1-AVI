#include "PID.hpp"

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki): 
    _dt(dt),
    _max(max), 
    _min(min), 
    _Kp(Kp), 
    _Kd(Kd), 
    _Ki(Ki), 
    _integral(0.0), 
    _prevError(0.0)
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
    double error = _setpoint - input;                                   // P
    _integral += error * _dt;                                           // I
    double derivative = (error - _prevError) / _dt;                     // D

    // sum
    _prevError = error;
    double output = _Kp * error + _Ki * _integral + _Kd * derivative;

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

void PID::reset()
{
    _integral = 0.0;
    _prevError = 0.0;
}


PID::~PID()
{
}