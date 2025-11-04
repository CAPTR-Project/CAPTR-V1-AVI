#include "mount_lib.hpp"

namespace tvc_mount_lib {

    TVC_mount::TVC_mount(int pitch_pin, double scaling_pitch_, double offset_pitch_, double pitch_limit,
                  int yaw_pin, double scaling_yaw_, double offset_yaw_, double yaw_limit) {
        this->scaling_pitch = scaling_pitch_;
        this->scaling_yaw = scaling_yaw_;

        this->offset_pitch = offset_pitch_;
        this->offset_yaw = offset_yaw_;

        this->pitch_limit = pitch_limit;
        this->yaw_limit = yaw_limit;

        this->servo_pitch.attach(pitch_pin, 1000, 2000);
        this->servo_yaw.attach(yaw_pin, 1000, 2000);
        this->servo_pitch.writeMicroseconds(1500);
        this->servo_yaw.writeMicroseconds(1500);
        

    }

    void TVC_mount::move_mount(double pitch_rad, double yaw_rad) {
        if (pitch_rad > this->pitch_limit) pitch_rad = this->pitch_limit;
        if (pitch_rad < -this->pitch_limit) pitch_rad = -this->pitch_limit;
        if (yaw_rad > this->yaw_limit) yaw_rad = this->yaw_limit;
        if (yaw_rad < -this->yaw_limit) yaw_rad = -this->yaw_limit;

        this->pitch_cmd_rad_ = pitch_rad;
        this->yaw_cmd_rad_ = yaw_rad;

        int pitch_us = 1500 + ((int) ((pitch_rad + offset_pitch) * this->scaling_pitch / (M_PI / 4) * 500));
        this->servo_pitch.writeMicroseconds(pitch_us);
        int yaw_us = 1500 + ((int)((yaw_rad + offset_yaw) * this->scaling_yaw / (M_PI / 4) * 500));
        this->servo_yaw.writeMicroseconds(yaw_us);

        // Serial.println("Mount Pitch CMD: " + String(pitch_rad) + " Yaw CMD: " + String(yaw_rad));
    }

    void TVC_mount::preflight_test() {
        busy = true;
        xTaskCreate(preflight_test_task, "TVC Preflight Test", 1000, this, 1, &preflight_test_task_handle);
    }

    void TVC_mount::preflight_test_task(void* mount_ptr_void) {
        TVC_mount* mount_ptr = static_cast<TVC_mount*>(mount_ptr_void);
        // Test pitch
        for (int i = 0; i < 2; i++) {
            mount_ptr->move_mount(mount_ptr->pitch_limit, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            mount_ptr->move_mount(-mount_ptr->pitch_limit, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        mount_ptr->move_mount(0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Test yaw
        for (int i = 0; i < 2; i++) {
            mount_ptr->move_mount(0, mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
            mount_ptr->move_mount(0, -mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        mount_ptr->move_mount(0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Test pitch and yaw
        for (int i = 0; i < 1; i++) {
            mount_ptr->move_mount(mount_ptr->pitch_limit, mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
            mount_ptr->move_mount(-mount_ptr->pitch_limit, -mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        mount_ptr->move_mount(0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        for (int i = 0; i < 1; i++) {
            mount_ptr->move_mount(-mount_ptr->pitch_limit, mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
            mount_ptr->move_mount(mount_ptr->pitch_limit, -mount_ptr->yaw_limit);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        mount_ptr->move_mount(0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));

        mount_ptr->busy = false;

        vTaskDelete(mount_ptr->preflight_test_task_handle);
        return;
    }
}