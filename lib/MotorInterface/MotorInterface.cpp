#include "MotorInterface.h"

#ifndef PWM_DRIVER
Adafruit_PWMServoDriver Motor::pwmDriver = Adafruit_PWMServoDriver();
#endif

void Motor::begin(){
    pwmDriver.begin();
    pwmDriver.setPWMFreq(1600);  // This is the maximum PWM frequency
}

void Motor::update(){
    // Apply inversion if set
    if (inverted) currentRawSpeed = -currentRawSpeed;
    currentRawSpeed = constrain(currentRawSpeed, -4095, 4095);
    
    // Set PWM values based on direction
    if (currentRawSpeed > 0) {
        pwmDriver.setPWM(motorA, 0, currentRawSpeed);
        pwmDriver.setPWM(motorB, 0, 0);
    } else if (currentRawSpeed < 0) {
        pwmDriver.setPWM(motorA, 0, 0);
        pwmDriver.setPWM(motorB, 0, -currentRawSpeed);
    } else {
        pwmDriver.setPWM(motorA, 0, 0);
        pwmDriver.setPWM(motorB, 0, 0);
    }
}


Motor::Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB) {
    motorA = motorPortA;
    motorB = motorPortB;
    encA = encoderPortA;
    encB = encoderPortB;
}

Motor Motor::isInverted(bool inverted){
    Motor::inverted = inverted;
    return *this;
}

int* Motor::setRawSpeed(int speed) {
    currentRawSpeed = speed;
    return &currentRawSpeed;
}





void Motor::printStatus() {
    Serial.print("Current Speed: \t");
    Serial.println(currentRawSpeed);
    Serial.print("Current Position: \t");
    Serial.println(currentPosition);
    Serial.print("Current RPM: \t");
    Serial.println(current_rpm);
}

