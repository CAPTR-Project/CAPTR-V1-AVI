#include "sensors_setup.hpp"

namespace sensors_lib {
    /*
    bool initBMP(bmp390_handle_s* bmp, bmp390_address_t i2cAddr, TwoWire* I2CBus, bmp390_odr_t datarate, void (*alt_isr)(), uint8_t isr_pin) {
        // if (!bmp->begin_I2C(i2cAddr, I2CBus)) {
        //     Serial.println("ERROR: Failed to find BMP390 sensor");
        //     return false;
        // }
        // Serial.println("BMP3 sensor found");
        // bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        // bmp->setPressureOversampling(BMP3_OVERSAMPLING_16X);
        // bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        // bmp->setOutputDataRate(BMP3_ODR_200_HZ);

        DRIVER_BMP390_LINK_INIT(bmp, bmp390_handle_t);
        DRIVER_BMP390_LINK_IIC_INIT(bmp, bmp390_interface_iic_init);
        DRIVER_BMP390_LINK_IIC_DEINIT(bmp, bmp390_interface_iic_deinit);
        DRIVER_BMP390_LINK_IIC_READ(bmp, bmp390_interface_iic_read);
        DRIVER_BMP390_LINK_IIC_WRITE(bmp, bmp390_interface_iic_write);
        DRIVER_BMP390_LINK_SPI_INIT(bmp, bmp390_interface_spi_init);
        DRIVER_BMP390_LINK_SPI_DEINIT(bmp, bmp390_interface_spi_deinit);
        DRIVER_BMP390_LINK_SPI_READ(bmp, bmp390_interface_spi_read);
        DRIVER_BMP390_LINK_SPI_WRITE(bmp, bmp390_interface_spi_write);
        DRIVER_BMP390_LINK_DELAY_MS(bmp, bmp390_interface_delay_ms);
        DRIVER_BMP390_LINK_DEBUG_PRINT(bmp, bmp390_interface_debug_print);
        
        int res;
        
        
        res = bmp390_set_interface(bmp, bmp390_interface_t::BMP390_INTERFACE_IIC);
        res = bmp390_set_addr_pin(bmp, i2cAddr);

        res = bmp390_init(bmp);

        res = bmp390_set_iic_watchdog_timer(bmp, BMP390_BOOL_FALSE);
        res = bmp390_set_iic_watchdog_period(bmp, BMP390_IIC_WATCHDOG_PERIOD_1P25_MS);
        res = bmp390_set_fifo(bmp, BMP390_BOOL_FALSE);
        res = bmp390_set_interrupt_pin_type(bmp, BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL);
        res = bmp390_set_interrupt_active_level(bmp, BMP390_INTERRUPT_ACTIVE_LEVEL_HIGHER);
        res = bmp390_set_latch_interrupt_pin_and_interrupt_status(bmp, BMP390_BOOL_FALSE);
        res = bmp390_set_interrupt_fifo_watermark(bmp, BMP390_BOOL_FALSE);
        res = bmp390_set_interrupt_fifo_full(bmp, BMP390_BOOL_FALSE);
        res = bmp390_set_interrupt_data_ready(bmp, BMP390_BOOL_TRUE);
        res = bmp390_set_temperature(bmp, BMP390_BOOL_TRUE);
        res = bmp390_set_pressure(bmp, BMP390_BOOL_TRUE);
        res = bmp390_set_temperature_oversampling(bmp, bmp390_oversampling_t::BMP390_OVERSAMPLING_x1);
        res = bmp390_set_pressure_oversampling(bmp, bmp390_oversampling_t::BMP390_OVERSAMPLING_x4);
        res = bmp390_set_filter_coefficient(bmp, bmp390_filter_coefficient_t::BMP390_FILTER_COEFFICIENT_3);
        res = bmp390_set_mode(bmp, bmp390_mode_t::BMP390_MODE_NORMAL_MODE);
        res = bmp390_set_odr(bmp, datarate);
        bmp390_mode_t mode;
        res = bmp390_get_mode(bmp, &mode);
        if (mode != bmp390_mode_t::BMP390_MODE_NORMAL_MODE) return false;

        if (res > 0) return false;

        vPortEnterCritical();

        attachInterrupt(digitalPinToInterrupt(isr_pin), alt_isr, arduino::RISING);

        (*alt_isr)(); // read once to clear interrupt

        vPortExitCritical();

        return true;
    }*/

    bool initBMP(Adafruit_BMP3XX* bmp, uint8_t i2cAddr, TwoWire* I2CBus, uint8_t dataRate,
                 void (*baro_isr)(), uint8_t baro_isr_pin) {
        if (!bmp->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find BMP390 sensor");
            return false;
        }

        bool res;

        res = bmp->setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        res = bmp->setPressureOversampling(BMP3_OVERSAMPLING_8X);
        res = bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
        res = bmp->setOutputDataRate(BMP3_ODR_50_HZ);

        if (!res) return false;

        I2CBus->beginTransmission(i2cAddr);
        I2CBus->write(0x19);
        I2CBus->write(0b01000010);
        I2CBus->endTransmission();

        vPortEnterCritical();

        attachInterrupt(digitalPinToInterrupt(baro_isr_pin), baro_isr, arduino::RISING);

        bmp->readAltitude(1013.25);

        vPortExitCritical();

        return true;
    }

    bool initIMU_LSM6DSO32(Adafruit_LSM6DSO32* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate,
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin) {
        
        if (!imu->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find LSM6DS chip");
            return false;
        }

        Serial.println("LSM6DS Found!");

        imu->setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
        imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        imu->setAccelDataRate(dataRate);
        imu->setGyroDataRate(dataRate);

        imu->getGyroRange();
        imu->getAccelRange();

        vPortEnterCritical();

        imu->configInt1(false, true, false); // enable interrupt on gyroscope data ready
        imu->configInt2(false, false, true); // enable interrupt on accelerometer data ready
        imu->configIntOutputs(false, false); // set to active high and push-pull


        attachInterrupt(digitalPinToInterrupt(accel_isr_pin), imu_isr, arduino::RISING);
        attachInterrupt(digitalPinToInterrupt(gyro_isr_pin), gyro_isr, arduino::RISING);

        // (*imu_isr)(); // read once to clear interrupt
        // (*gyro_isr)(); // read once to clear interrupt
        float x, y, z;
        imu->readAcceleration(x, y, z);
        imu->readGyroscope(x, y, z);

        vPortExitCritical();

        return true;
    }

    bool initIMU_LSM6DS(Adafruit_LSM6DS* imu, uint8_t i2cAddr, TwoWire* I2CBus, lsm6ds_data_rate_t dataRate,
                 void (*imu_isr)(), uint8_t accel_isr_pin, 
                 void (*gyro_isr)(), uint8_t gyro_isr_pin) {
        
        if (!imu->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find LSM6DS chip");
            return false;
        }

        Serial.println("LSM6DS Found!");

        imu->setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
        imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
        imu->setAccelDataRate(dataRate);
        imu->setGyroDataRate(dataRate);

        imu->getGyroRange();
        imu->getAccelRange();

        vPortEnterCritical();

        imu->configInt1(false, true, false); // enable interrupt on gyroscope data ready
        imu->configInt2(false, false, true); // enable interrupt on accelerometer data ready
        imu->configIntOutputs(false, false); // set to active high and push-pull


        attachInterrupt(digitalPinToInterrupt(accel_isr_pin), imu_isr, arduino::RISING);
        attachInterrupt(digitalPinToInterrupt(gyro_isr_pin), gyro_isr, arduino::RISING);

        // (*imu_isr)(); // read once to clear interrupt
        // (*gyro_isr)(); // read once to clear interrupt
        float x, y, z;
        imu->readAcceleration(x, y, z);
        imu->readGyroscope(x, y, z);

        vPortExitCritical();

        return true;
    }

    bool initMag(Adafruit_LIS3MDL* lis_mag, uint8_t i2cAddr, TwoWire* I2CBus, lis3mdl_dataRate_t datarate, 
                 void (*mag_isr)(), uint8_t mag_isr_pin) {
        if (!lis_mag->begin_I2C(i2cAddr, I2CBus)) {
            Serial.println("ERROR: Failed to find LIS3MDL chip");
        }

        Serial.println("LIS3MDL Found!");

        lis_mag->reset();
        lis_mag->setRange(LIS3MDL_RANGE_16_GAUSS);
        lis_mag->setDataRate(datarate);
        lis_mag->setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        lis_mag->setOperationMode(LIS3MDL_CONTINUOUSMODE);

        vPortEnterCritical();

        attachInterrupt(digitalPinToInterrupt(mag_isr_pin), mag_isr, arduino::RISING);

        lis_mag->read(); // read once to clear interrupt

        vPortExitCritical();

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