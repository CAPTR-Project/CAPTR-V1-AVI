#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include "quaternion.h"  // custom quaternion library

class QuaternionPID
{
public:
    PID(double dt, Eigen::Vector3f max, Eigen::Vector3f min, Eigen::Vector3f Kp, Eigen::Vector3f Kd, Eigen::Vector3f Ki);        // has to be same name as class because it is a constructor like init in python
    ~PID();

    // PID functions
    Eigen::Vector3f update(Quaternion input);
    void setDt(double dt);
    void setSetpoint(Quaternion setpoint);
    void reset();

private:
    Eigen::Vector3f Kp_, Ki_, Kd_;
    Eigen::Vector3f min_, max_;
    Quaternion setpoint_;
    Quaternion error_;
    Quaternion prevError_;
    Quaternion derivative_;
    Quaternion integral_;
    double dt_;
};

#endif
