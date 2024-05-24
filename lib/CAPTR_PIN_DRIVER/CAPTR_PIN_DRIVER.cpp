#include "CAPTR_PIN_DRIVER.hpp"

HwPin controller_pins[COUNT] = {

    [HW_PIN_IMU] = {
        .pin_number = 7,
        .output = true,
        .physical_state = false,
    },
	
	[HW_PIN_BARO] = {
        .pin_number = 7,
        .output = true,
        .physical_state = false,
    },

    [HW_PIN_SERVO_X] = {
        .pin_number = 7,
        .output = true,
        .physical_state = false,
    },

    [HW_PIN_SERVO_Y] = {
        .pin_number = 7,
        .output = true,
        .physical_state = false,
    },
}

void HwSetupPins(void){
  for (int i = 0; i < COUNT; i++){
    if (controller_pins[i].is_output) {
      pinMode(controller_pins[i].pin_number, OUTPUT);
      digitalWrite(controller_pins[i].pin_number, controller_pins[i].physical_state);
    } else {
      pinMode(controller_pins[i].pin_number, INPUT);
    }
  }
}

bool HwDigitalRead(ControllerPin x){
  bool state = false;
  if (x < COUNT) {
    state = digitalRead(controller_pins[x].pin_number);
    controller_pins[x].physical_state = state;
  }
  
  return state;
}

void HwDigitalWrite(ControllerPin x, bool physical_state){
  if (x < COUNT){
    digitalWrite(controller_pins[x].pin_number, physical_state);
    controller_pins[x].physical_state = physical_state;
  }
}

int HwAnalogRead(ControllerPin x){
  if (x < COUNT){
    return analogRead(controller_pins[x].pin_number);
  }
}
void HwAnalogWrite(ControllerPin x, int value){
  if (x < COUNT){
    digitalWrite(controller_pins[x].pin_number, value);
  }
}