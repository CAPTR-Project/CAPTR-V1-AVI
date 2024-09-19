#include "CAPTR_PIN_DRIVER.hpp"

HwPin controller_pins[COUNT] = {

  [HW_PIN_SERVO_X] = {
    .pin_number = 14,
    .output = true,
    .physical_state = false,
  },

  [HW_PIN_SERVO_Y] = {
    .pin_number = 15,
    .output = true,
    .physical_state = false,
  },

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

void HwSetupPins(void){
  for (int i = 0; i < COUNT; i++){
    if (controller_pins[i].output) {
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
  return -1;
}
void HwAnalogWrite(ControllerPin x, int value){
  if (x < COUNT){
    digitalWrite(controller_pins[x].pin_number, value);
  }
}