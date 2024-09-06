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

// const int FlashChip = 1; // Chip select pin for flash memory CHANGE WHEN IMPLEMENTED!!!!!

// check if individual sensor data is valid
bool isValidAccelData(const sensor_msgs::AccelMsg& accel) {
    const float MAX_ACCEL = 100.0f; // m/s^2 change to accelerometer max value
    return (std::abs(accel.x) <= MAX_ACCEL &&
            std::abs(accel.y) <= MAX_ACCEL &&
            std::abs(accel.z) <= MAX_ACCEL);
}

bool isValidGyroData(const sensor_msgs::GyroMsg& gyro) {
    const float MAX_GYRO = 1000.0f; // deg/s change to gyroscope max value
    return (std::abs(gyro.x) <= MAX_GYRO &&
            std::abs(gyro.y) <= MAX_GYRO &&
            std::abs(gyro.z) <= MAX_GYRO);
}

bool isValidBaroData(const sensor_msgs::BaroMsg& baro) {
    const float MIN_PRESSURE = 800.0f; // hPa change to barometer min value
    const float MAX_PRESSURE = 1100.0f; // hPa change to barometer max value
    return (baro.pressure >= MIN_PRESSURE && baro.pressure <= MAX_PRESSURE);
}

bool isValidMagData(const sensor_msgs::MagMsg& mag) {
    const float MAX_MAG = 1000.0f; // µT change to magnetometer max value
    return (std::abs(mag.x) <= MAX_MAG &&
            std::abs(mag.y) <= MAX_MAG &&
            std::abs(mag.z) <= MAX_MAG);
}

// Log data to SerialFlash
void logData(const SensorLog& data) {
    // Write data to flash memory
    const char *filename = "datalog.bin";

    if (!SerialFlash.exists(filename)) {
        if (!SerialFlash.create(filename, sizeof(SensorLog) * 10000)) { // Space for 10 000 logs
            error_state = ErrorState::FLASH_CREATE;
            return;
        }
    }

    SerialFlashFile flashFile = SerialFlash.open(filename);

    if (!flashFile) {
        error_state = ErrorState::FLASH_CREATE;
        return;
    } else {
        // find end of file
        flashFile.seek(flashFile.size());
        
        // write data to flash
        if (flashFile.write(&data, sizeof(SensorLog)) != sizeof(SensorLog)) {
            error_state = ErrorState::FLASH_WRITE;
        }

        flashFile.close();
   
    }
}

void datalogger_thread(void*) {
    // init flash 
    if (!SerialFlash.begin(FLASH_CHIP)) {
        error_state = ErrorState::FLASH_INIT;
        return;
    }

    // Data logger
    TickType_t xLastWakeTime = xTaskGetTickCount();
    BaseType_t xWasDelayed;
    
    // Create a file for logging if not already created (use library)

    //handle file open error

    while (1) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        
        // Read sensor data
        long long currentTimestamp = msElapsed.load();

        SensorLog currentLog;
        currentLog.baro = baro_data;
        currentLog.accel = accel_data;
        currentLog.gyro = gyro_data;
        currentLog.mag = mag_data;
        currentLog.timestamp = currentTimestamp;

        // OR individually
        // sensor_msgs::BaroMsg local_baro_data = baro_data;
        // sensor_msgs::AccelMsg local_accel_data = accel_data;
        // sensor_msgs::GyroMsg local_gyro_data = gyro_data;
        // sensor_msgs::MagMsg local_mag_data = mag_data;

        // log data to flash
        logData(currentLog);

        if (xWasDelayed) {
            // Log error
            error_state = ErrorState::LOGGING;

            if (!isValidAccelData(currentLog.accel)) {
                error_state = ErrorState::ACCEL;
            } else if (!isValidGyroData(currentLog.gyro)) {
                error_state = ErrorState::GYRO;
            } else if (!isValidBaroData(currentLog.baro)) {
                error_state = ErrorState::BARO;
            } else if (!isValidMagData(currentLog.mag)) {
                error_state = ErrorState::MAG;
            }
        }
    }
}