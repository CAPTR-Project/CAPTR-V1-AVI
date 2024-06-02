#include <stdio.h>

// pid controller

class PID
{
    public:
        PID();
        ~PID();
        void setGains(float kp, float ki, float kd);
        void setLimits(float min, float max);
        void setDt(float dt);
        void setSetpoint(float setpoint);
        float update(float input);
        void reset();
    private:
        float kp, ki, kd;
        float min, max;
        float dt;
        float setpoint;
        float integral;
        float prevError;
};

