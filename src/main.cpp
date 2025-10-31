#include <Arduino.h>
#include <MotorInterface.h>

Motor motor(12, 13, 2, 7);
int i = -4095;
bool up = true;
unsigned long lastTime = 0;
unsigned long lastProcessTime;

void setup() {
    Motor::begin();
    motor.CountsPerRevolution = 1400; // Set according to your encoder specification
    lastProcessTime = 0;
    attachInterrupt(digitalPinToInterrupt(motor.encA), [](){ motor.updateCounts();}, RISING);
}


void loop() {
  if(up) i++;
  else i--;
  if(i >= 4095) up = false;
  if(i <= -4095) up = true;

  motor.setRawSpeed(up? -4096: 4096);

  
  motor.update(lastProcessTime, millis());
  lastProcessTime = millis();

  if(millis() - lastTime > 1000){
    lastTime = millis();
    motor.printStatus();
  }
}