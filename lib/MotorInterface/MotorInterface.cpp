#include "MotorInterface.h"

#ifndef PWM_DRIVER
Adafruit_PWMServoDriver Motor::pwmDriver = Adafruit_PWMServoDriver();
#endif

void Motor::begin(){
    pwmDriver.begin();
    pwmDriver.setPWMFreq(400);  // This is the maximum PWM frequency

}

void Motor::update(unsigned long lastTime, unsigned long currentTime){
    unsigned long deltaTime = currentTime - lastTime; // in milliseconds

    // Calculate RPM
    int deltaCounts = currentCounts - lastCounts;
    current_rpm = (deltaCounts / CountsPerRevolution) / deltaTime; //rotations per ms
    current_rpm *= 1000; // rotations per second
    current_rpm *= 60; // rotations per minute
    lastCounts = currentCounts;



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

    pinMode(encA, INPUT);
    pinMode(encB, INPUT);
}

Motor Motor::isInverted(bool inverted){
    Motor::inverted = inverted;
    return *this;
}

int* Motor::setRawSpeed(int speed) {
    currentRawSpeed = speed;
    return &currentRawSpeed;
}



void Motor::updateCounts() {
    if (digitalRead(encA) == digitalRead(encB)) {
        currentCounts += inverted ? 1 : -1;  // Invert direction if flag is set
    } else {
        currentCounts += inverted ? -1 : 1;  // Invert direction if flag is set
    }
}

void Motor::printStatus() {
    Serial.print("Current Speed: \t");
    Serial.println(currentRawSpeed);
    Serial.print("Current Counts: \t");
    Serial.println(currentCounts);
    Serial.print("Current RPM: \t");
    Serial.println(current_rpm);
}

