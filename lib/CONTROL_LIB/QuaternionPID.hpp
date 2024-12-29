#ifndef QUATERNIONPID_HPP
#define QUATERNIONPID_HPP

#include <iostream>
#include "PID.hpp"  // custom PID library
#include "quaternion.h"  // custom quaternion library

class QuaternionPID
{
public:
    QuaternionPID(float dt, float max, float min, Eigen::Vector3d Kp, Eigen::Vector3d Ki, Eigen::Vector3d Kd)
    : qpidX_(dt, max, min, Kp(0), Ki(0), Kd(0)),
    qpidY_(dt, max, min, Kp(1), Ki(1), Kd(1)),
    qpidZ_(dt, max, min, Kp(2), Ki(2), Kd(2)) {}

    // calc (calc stands for calculate if you just joined) the error and return pid computed values in vector3
    Eigen::Vector3d compute(UnitQuaternion attSetpoint, UnitQuaternion currentAtt) {
        // calculate error instead of passing into pid first
        errorQuat_ = attSetpoint * currentAtt.conjugate();
        return {
            qpidX_.run(errorQuat_.v_1, 0),
            qpidY_.run(errorQuat_.v_2, 0),
            qpidZ_.run(errorQuat_.v_3, 0)
        };
    }

private:
    Quaternion errorQuat_;
    PID qpidX_;
    PID qpidY_;
    PID qpidZ_;
};

#endif
