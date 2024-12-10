#include "mount_lib.hpp"

namespace tvc_mount_lib {

    TVC_mount::TVC_mount(int x_pin, double scaling_pitch_, double offset_pitch_, int y_pin, double scaling_yaw_, double offset_yaw_) {
        this->scaling_pitch = scaling_pitch_;
        this->scaling_yaw = scaling_yaw_;
        this->offset_pitch = offset_pitch_;
        this->offset_yaw = offset_yaw_;
        this->servo_pitch.attach(x_pin);
        this->servo_yaw.attach(y_pin);
    }

    void TVC_mount::move_mount(double x_rad, double y_rad) {
        double pitch_us = 1500 + ((x_rad * this->scaling_pitch + offset_pitch) / (M_PI / 4) * 500);
        this->servo_pitch.writeMicroseconds(pitch_us);
        double yaw_us = 1500 + ((y_rad * this->scaling_yaw + offset_yaw) / (M_PI / 4) * 500);
        this->servo_yaw.writeMicroseconds(yaw_us);
    }
}