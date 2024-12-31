#include "threads/gps_thread.hpp"

namespace gps_thread {

    void gps_thread(void*) {

        sensors_lib::initGPS(&gps__, &GPS_SERIAL_PORT, GPS_BAUDRATE);

        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;
        while (1) {
            while (GPS_SERIAL_PORT.available() > 0)
            gps__.encode(GPS_SERIAL_PORT.read());
        }

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(1000/GPS_FREQUENCY)));
        if (!xWasDelayed) {
            xSemaphoreTake(serial_port_mutex__, 0);
            Serial.println("GPS loop delayed");
            xSemaphoreGive(serial_port_mutex__);
            error_state_ = ErrorState::GPS;
        }
    }

} // namespace gps_thread