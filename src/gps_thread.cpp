#include "threads/gps_thread.hpp"

namespace gps_thread {

    void gps_thread(void*) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;
        while (1) {
            while (GPS_SERIAL_PORT.available() > 0)
            gps__.encode(GPS_SERIAL_PORT.read());
        }

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(1000/GPS_FREQUENCY)));
        if (!xWasDelayed) {
            error_state_ = ErrorState::GPS;
        }
    }

} // namespace gps_thread