#include "gps.hpp"

namespace sensors::gps {

    void gps_task(void*) {

        GPS_SERIAL_PORT.begin(GPS_BAUDRATE);

        TickType_t xLastWakeTime = xTaskGetTickCount();
        BaseType_t xWasDelayed;
        while (1) {
            while (GPS_SERIAL_PORT.available() > 0)
            gps_.encode(GPS_SERIAL_PORT.read());
        }

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(1000/GPS_FREQUENCY)));
        if (!xWasDelayed) {
            Serial.println("GPS loop delayed");
            // state_manager::setError(ErrorState::GPS);
        }
    }

    void gpsInit() {
        // Initialize the GPS thread
        xTaskCreate(gps_task, "GPS Thread", 2048, NULL, 1, NULL);
        Serial.println("GPS thread created");
    }

} // namespace gps_task