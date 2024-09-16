#include "sensors_setup.hpp"

namespace sensors_lib {
    bool initBMP(Adafruit_BMP3XX* bmp, uint8_t i2cAddr, TwoWire* I2CBus, void (*alt_isr)(), uint8_t isr_pin) {
        if (!bmp->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find BMP390 sensor");
            return false;
        }
        Serial.println("BMP3 sensor found");
        bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp->setPressureOversampling(BMP3_OVERSAMPLING_16X);
        bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp->setOutputDataRate(BMP3_ODR_200_HZ);

        return true;
    }

    bool initIMU(Adafruit_LSM6DS* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate,
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin) {
        
        if (!imu->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find LSM6DS chip");
            return false;
        }

        Serial.println("LSM6DS Found!");

        imu->setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
        imu->setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
        imu->setAccelDataRate(dataRate);
        imu->setGyroDataRate(dataRate);

        imu->configIntOutputs(true, false); // set to active high and push-pull
        imu->configInt1(false, true, false); // enable interrupt on gyroscope data ready
        imu->configInt2(false, false, true); // enable interrupt on accelerometer data ready

        attachInterrupt(digitalPinToInterrupt(accel_isr_pin), imu_isr, arduino::RISING);
        attachInterrupt(digitalPinToInterrupt(gyro_isr_pin), gyro_isr, arduino::RISING);

        return true;
    }

    bool initMag(Adafruit_LIS3MDL* lis_mag, uint8_t i2cAddr, TwoWire* I2CBus, lis3mdl_dataRate_t datarate, 
                 void (*mag_isr)(), uint8_t mag_isr_pin) {
        if (!lis_mag->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find LIS3MDL chip");
            return false;
        }

        Serial.println("LIS3MDL Found!");

        lis_mag->setRange(LIS3MDL_RANGE_16_GAUSS);
        lis_mag->setDataRate(datarate);
        lis_mag->setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        lis_mag->setOperationMode(LIS3MDL_CONTINUOUSMODE);

        attachInterrupt(mag_isr_pin, mag_isr, arduino::RISING);

        return true;
    }

    void initRadio() {
        // if (!rf95_driver.init()) {
        //     Serial.println("ERROR: LoRa radio init failed");
        // }
        // Serial.println("LoRa radio init OK!");
        // rf95_driver.setFrequency(915.0);
        // rf95_driver.setTxPower(15, false);

    }

    void initGPS(TinyGPSPlus* gps, HardwareSerial* gpsSerial, uint32_t baudrate) {
        gpsSerial->begin(baudrate);
    }
}