/*

 ██████  █████  ██████  ████████ ██████
██      ██   ██ ██   ██    ██    ██   ██
██      ███████ ██████     ██    ██████ 
██      ██   ██ ██         ██    ██   ██  
 ██████ ██   ██ ██         ██    ██   ██ 

File: datalogger_thread.cpp
Auth: Joseph Mi
Desc: Source file datalogger thread of BaroMsg, AccelMsg, GyroMsg, and mag_data

*/

#include "threads/datalogger_thread.hpp"

namespace datalogger_thread {

// const int FlashChip = 1; // Chip select pin for flash memory CHANGE WHEN IMPLEMENTED!!!!!

// Log data to SerialFlash
void logData(const datalogger_thread::SensorLog& data) {
    // Write data to flash memory
    const char *filename = "datalog.bin";

    if (!SerialFlash.exists(filename)) {
        if (!SerialFlash.create(filename, sizeof(datalogger_thread::SensorLog) * 10000)) { // Space for 10 000 logs
            error_state_ = ErrorState::FLASH_CREATE;
            return;
        }
    }

    SerialFlashFile flashFile = SerialFlash.open(filename);

    if (!flashFile) {
        error_state_ = ErrorState::FLASH_CREATE;
        return;
    } else {
        // find end of file
        flashFile.seek(flashFile.size());
        
        // write data to flash
        if (flashFile.write(&data, sizeof(datalogger_thread::SensorLog)) != sizeof(datalogger_thread::SensorLog)) {
            error_state_ = ErrorState::FLASH_WRITE;
        }

        flashFile.close();
   
    }
}

void datalogger_thread(void*) {
    // init flash 
    if (!SerialFlash.begin(SPI, FLASH_CHIP)) {
        error_state_ = ErrorState::FLASH_INIT;
    }

    // Data logger
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    
    // Create a file for logging if not already created (use library)

    //handle file open error

    while (true) {
        
        // Read sensor data
        uint32_t currentTimestamp = msElapsed;

        datalogger_thread::SensorLog currentLog;
        if (xSemaphoreTake(baro_data__.ready, 0) == pdTRUE) {
            currentLog.baro = baro_data__;
            xSemaphoreGive(baro_data__.ready);
        }
        if (xSemaphoreTake(accel_data__.ready, 0) == pdTRUE) {
            currentLog.accel = accel_data__;
            xSemaphoreGive(accel_data__.ready);
        }
        if (xSemaphoreTake(gyro_data__.ready, 0) == pdTRUE) {
            currentLog.gyro = gyro_data__;
            xSemaphoreGive(gyro_data__.ready);
        }
        if (xSemaphoreTake(mag_data__.ready, 0) == pdTRUE) {
            currentLog.mag = mag_data__;
            xSemaphoreGive(mag_data__.ready);
        }
        
        currentLog.timestamp = currentTimestamp;

        // OR individually
        // sensor_msgs::BaroMsg local_baro_data = baro_data;
        // sensor_msgs::AccelMsg local_accel_data = accel_data;
        // sensor_msgs::GyroMsg local_gyro_data = gyro_data;
        // sensor_msgs::MagMsg local_mag_data = mag_data;

        // log data to flash
        logData(currentLog);

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(int(round(1000/LOGGING_FREQUENCY))));
        if (xWasDelayed) {
            // Log error
            // error_state_ = ErrorState::LOGGING;
        }
    }
}

} // namespace datalogger_thread