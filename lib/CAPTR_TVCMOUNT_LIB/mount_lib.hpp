#include <Arduino.h>
// #include "Servo.h"
#include "custom_PWMServo.h"

#include "arduino_freertos.h"
#include <atomic>

namespace tvc_mount_lib {
    class TVC_mount {

    public:
        TVC_mount(int pitch_pin, double scaling_pitch_, double offset_pitch_, double pitch_limit,
                  int yaw_pin, double scaling_yaw_, double offset_yaw_, double yaw_limit);

        PWMServo servo_pitch;
        PWMServo servo_yaw;
        double offset_pitch = 0;
        double offset_yaw = 0;
        double scaling_pitch = 0;
        double scaling_yaw = 0;
        double pitch_limit = 0;
        double yaw_limit = 0;

        bool busy = false;


        void move_mount(double x_rad, double y_rad);

        void preflight_test();

    private:
        TaskHandle_t preflight_test_task_handle;
        static void preflight_test_task(void* mount_ptr);

    };
}