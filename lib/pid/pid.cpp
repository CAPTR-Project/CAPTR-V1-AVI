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
    _error = _setpoint - input;                                   // P
    _integral += _error * _dt;                                           // I
    _derivative = (_error - _prevError) / _dt;                     // D

    // sum
    _prevError = error;
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

void PID::reset()
{
    _integral = 0.0;
    _prevError = 0.0;
}


PID::~PID()
{
}

// test pid

int main(){
    PID pid(0.1, 100, -100, 0.1, 0.01, 0.5);
    pid.setSetpoint(10);
    double output = pid.update(5);
    std::cout << "Output: " << output << std::endl;     // Output: 0.5
    return 0;
}