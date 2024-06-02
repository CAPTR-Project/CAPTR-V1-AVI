#include "main.hpp"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("hi");

  initIMU(LSM6DS_I2CADDR_DEFAULT, &Wire);
  initMag(LIS3MDL_I2CADDR_DEFAULT, &Wire);
  // initBMP(BMP390_CHIP_ID, &Wire1);
}

void loop() {
  // Serial.println("live");
  // put your main code here, to run repeatedly:
  // double altitude = bmp.readAltitude(1013.25);
  
  lsm_accel->getEvent(&accelMainData);
  lsm_gyro->getEvent(&gyroData);
  lis_mag.getEvent(&magData);


  // Serial.println("Altitude:");
  // Serial.println(altitude);

  // Serial.println("Accel:");
  Serial.printf("Accel: %05.2f ", accelMainData.acceleration.x);
  Serial.printf("%05.2f ", accelMainData.acceleration.y);
  Serial.printf("%05.2f\n", accelMainData.acceleration.z);

  // Serial.println("Gyro:");
  Serial.printf("Gyro: %05.2f ", gyroData.gyro.heading);
  Serial.printf("%05.2f ", gyroData.gyro.pitch);
  Serial.printf("%05.2f\n", gyroData.gyro.roll);

  // Serial.println("Mag:");
  Serial.printf("Mag: %05.2f ", magData.magnetic.x);
  Serial.printf("%05.2f ", magData.magnetic.y);
  Serial.printf("%05.2f\n\n", magData.magnetic.z);

  delay(100);
}

// put function definitions here:
void initBMP(uint8_t i2cAddr, TwoWire* I2CBus){
  if (!bmp.begin_I2C(i2cAddr, &Wire)) {
    Serial.println("ERROR: Failed to find BMP390 sensor");
    return;
  }
  Serial.println("BMP3 sensor found");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}

void initIMU(uint8_t i2cAddr, TwoWire* I2CBus) {
  if (!imu.begin_I2C(i2cAddr, I2CBus)) {
    Serial.println("ERROR: Failed to find LSM6DS chip");
    return;
  }

  Serial.println("LSM6DS Found!");

  // Set to 2G range and 26 Hz update rate
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);

  lsm_accel = imu.getAccelerometerSensor();
  lsm_accel->printSensorDetails();

  lsm_gyro = imu.getGyroSensor();
  lsm_gyro->printSensorDetails();
}

void initMag(uint8_t i2cAddr, TwoWire* I2CBus) {
  if (!lis_mag.begin_I2C(i2cAddr, I2CBus)) {
    Serial.println("ERROR: Failed to find LIS3MDL chip");
    return;
  }

  Serial.println("LIS3MDL Found!");

  lis_mag.setRange(LIS3MDL_RANGE_16_GAUSS);
  lis_mag.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis_mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis_mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

void initRadio() {
  if (!rf95_driver.init()) {
    Serial.println("ERROR: LoRa radio init failed");
  }
  Serial.println("LoRa radio init OK!");
  rf95_driver.setFrequency(915.0);
  rf95_driver.setTxPower(15, false);

}