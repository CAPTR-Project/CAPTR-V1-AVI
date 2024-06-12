#ifndef PID_HPP
#define PID_HPP

#include <iostream>

class PID
{
public:
    PID(double dt, double max, double min, double Kp, double Kd, double Ki, double intMax);        // has to be same name as class because it is a constructor like init in python
    ~PID();

    // PID functions
    double update(double input);
    void setDt(double dt);
    void setSetpoint(double setpoint);
    void reset(bool all);

private:
    double _Kp, _Ki, _Kd;
    double _min, _max;
    double _dt;
    double _setpoint;
    double _error;
    double _derivative;
    double _integral;
    double _prevError;
    double _integral_max;
};

#endif
