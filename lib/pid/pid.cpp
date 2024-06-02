#include "pid.hpp"

// pid class
using namespace std;
class PID
{
    public:
        initPID(double dt, double max, double min, double Kp, double Kd, double Ki);
        ~PID();
        double update(double input);

    private:
        double _Kp, _Ki, _Kd;
        double _min, _max;
        double _dt;
        double _setpoint;
        double _integral;
        double _prevError;
};

PID::initPID(double dt, double max, double min, double Kp, double Kd, double Ki) :
{
    _Kp(Kp);
    _Kd(Kd);
    _Ki(Ki);
    _max(max);
    _min(min);
    _dt(dt);
    return this;
}


void setDt(float dt)
{
    this->dt = dt;
}

void setSetpoint(float setpoint)
{
    this->setpoint = setpoint;
}

double update(double input)       // main PID loop
{

    float error = setpoint - input;                                 // proportional
    integral += error * dt;                                         // integral 
    float derivative = (error - prevError) / dt;                    // derivative

    prevError = error;                                              
    float output = kp * error + ki * integral + kd * derivative;

    // clamping to min max outputs (i assume this will be vectoring angles)
    if (output < min)
    {
        output = min;
    }
    else if (output > max)
    {
        output = max;
    }
    return output;
}

void reset()    // reset accumulated error and values for new setpoint 
{
    integral = 0.0;
    prevError = 0.0;
}