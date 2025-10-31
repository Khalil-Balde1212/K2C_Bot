#include <Arduino.h>
#include <MotorInterface.h>

Motor motor(12, 13, 2, 7);
void setup() {
    Motor::begin();
}

int i = -4095;
bool up = true;
void loop() {
  if(up) i++;
  else i--;
  if(i >= 4095) up = false;
  if(i <= -4095) up = true;

  motor.setRawSpeed(i);

  
  motor.update();
}