#include <Arduino.h>

#ifndef CAPTR_PIN_DRIVER
#define CAPTR_PIN_DRIVER

enum ControllerPin{
    
	HW_PIN_IMU,             // 7
    HW_PIN_BARO,            // 8
    HW_PIN_SERVO_X,         // 28  
    HW_PIN_SERVO_Y,         // 29

    COUNT
};

struct HwPin{
    const int pin_number;
    bool is_output;
};

extern HwPin controller_pins[COUNT];

void HwSetupPins(void);
bool HwDigitalRead(FrontControllerPin x);
void HwDigitalWrite(FrontControllerPin x, bool physical_state);
int HwAnalogRead(FrontControllerPin x);
void HwAnalogWrite(FrontControllerPin x, int value);

#endif
