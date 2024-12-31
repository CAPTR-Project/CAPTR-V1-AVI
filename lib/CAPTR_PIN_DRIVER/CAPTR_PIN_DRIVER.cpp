#include "CAPTR_PIN_DRIVER.hpp"

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