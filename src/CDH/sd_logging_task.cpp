#include "sd_logging_task.hpp"

namespace sd_logging {

    
void sd_logging_task(void*) {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card Mount Failed");
        // state_manager::setError(ErrorState::LOGGING);
        // vTaskDelete(NULL);
    }

    if (!SD.exists("itertrackfile.txt")) {
        itertrackfile = SD.open("itertrackfile.txt", FILE_WRITE);
        itertrackfile.println("0\n");
    } else {
        itertrackfile = SD.open("itertrackfile.txt", FILE_WRITE);
    }

    if (!itertrackfile) {
        Serial.println("Failed to open itertrackfile.txt");
        state_manager::setError(ErrorState::LOGGING);
        vTaskDelete(NULL);
    }

    // Read whole file in one operation to avoid per-char overhead
    itertrackfile.seek(0);
    size_t sz = itertrackfile.size();
    String s;
    if (sz > 0) {
        char buf[4];
        size_t got = itertrackfile.readBytes(buf, sz);
        buf[got] = '\0';
        s = String(buf);
    }
    
    int iter = s.toInt();
    iter++;

    itertrackfile.seek(0);
    itertrackfile.println(String(iter) + "\n");
    itertrackfile.close();

    String logfile_name = "/logfile_" + String(iter) + ".csv";;
    logfile = SD.open(logfile_name.c_str(), FILE_WRITE);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    while (1) {

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(1000 / LOGGING_FREQUENCY)));
        if (!xWasDelayed) {
            // state_manager::setError(ErrorState::CONTROL);
            Serial.println("Logging loop delayed");
        }
    }

}

}