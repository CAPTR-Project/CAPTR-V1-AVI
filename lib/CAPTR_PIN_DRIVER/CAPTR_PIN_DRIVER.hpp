#include <Arduino.h>

#ifndef CAPTR_PIN_DRIVER
#define CAPTR_PIN_DRIVER

enum ControllerPin{
    
    HW_PIN_GYRO_INT,        // 40
    HW_PIN_ACCEL_INT,       // 39
    HW_PIN_MAG_INT,         // 38
    HW_PIN_BARO_INT,        // 41

    COUNT
};

struct HwPin{
    const int pin_number;
    bool output;
    bool physical_state;
};


HwPin controller_pins[COUNT] = {

  [HW_PIN_GYRO_INT] = {
    .pin_number = 40,
    .output = false,
    .physical_state = false,
  },

  [HW_PIN_ACCEL_INT] = {
    .pin_number = 39,
    .output = false,
    .physical_state = false,
  },

  [HW_PIN_MAG_INT] = {
    .pin_number = 38,
    .output = false,
    .physical_state = false,
  },

  [HW_PIN_BARO_INT] = {
    .pin_number = 41,
    .output = false,
    .physical_state = false,
  },
};

void HwSetupPins(void);
bool HwDigitalRead(ControllerPin x);
void HwDigitalWrite(ControllerPin x, bool physical_state);
int HwAnalogRead(ControllerPin x);
void HwAnalogWrite(ControllerPin x, int value);

#endif
