/*

██████ ██████ ██████ ██████ ██████
██     ██  ██ ██  ██   ██   ██  ██
██     ██████ ██████   ██   ██████ 
██     ██  ██ ██       ██   ██ ██  
██████ ██  ██ ██       ██   ██  ██ 

File: baro.cpp
Auth: Yubo Wang
Desc: Source file for barometer data acquisition

*/

#include "baro.hpp"

namespace sensors::baro {
    
    void baroInit() {
        if (!bmp_.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1)) {
            Serial.println("ERROR: Failed to find BMP390 sensor");
        }

        bool res;

        res = bmp_.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        res = bmp_.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        res = bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
        res = bmp_.setOutputDataRate(BMP3_ODR_50_HZ);

        Wire1.beginTransmission(BMP3XX_DEFAULT_ADDRESS);
        Wire1.write(0x19);
        Wire1.write(0b01000010);
        Wire1.endTransmission();

        vPortEnterCritical();

        attachInterrupt(digitalPinToInterrupt(BARO_INT_PIN), baroDaqISR, arduino::RISING);

        bmp_.readAltitude(1013.25);

        vPortExitCritical();

        xTaskCreate(baroDaqThread, "Barometer", 2048, NULL, 1, &baro_taskHandle);
        Serial.println("Barometer thread started");
    }

    void baroDaqISR() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Send notification to control task on INDEX 0, unblocking the update(). 

        configASSERT( baro_taskHandle != NULL );
        vTaskNotifyGiveFromISR(baro_taskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    void baroDaqThread(void*) {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
            baroData_.alt_msl = bmp_.readAltitude(BARO_PRESSURE_ASL * 0.01);

            baroData_.alt_agl = baroData_.alt_msl - ground_altitude_offset_msl_;

            baroData_.pressure = bmp_.pressure;
        }
    }

}