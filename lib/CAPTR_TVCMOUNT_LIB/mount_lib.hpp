#include "Arduino.h"
#include "Servo.h"

#include "arduino_freertos.h"

namespace tvc_mount_lib {
    class TVC_mount {

    public:
        TVC_mount(int x_pin, double scaling_x_, double offset_pitch_, int y_pin, double scaling_y_, double offset_yaw_);

        Servo servo_pitch;
        Servo servo_yaw;
        double offset_pitch = 0;
        double offset_yaw = 0;
        double scaling_pitch = 0;
        double scaling_yaw = 0;

        void move_mount(double x_rad, double y_rad);

        void preflight_test();

    private:
        void preflight_test_task();

    };
}