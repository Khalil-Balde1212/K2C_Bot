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
    if (invertedMotor) currentRawSpeed = -currentRawSpeed;
    currentRawSpeed = constrain(currentRawSpeed, -4095, 4095);
    
    // Set PWM values based on direction
    if (currentRawSpeed > 0) {
        pwmDriver.setPWM(motorA, 0, currentRawSpeed);
        pwmDriver.setPWM(motorB, 0, 0);
    } else if (currentRawSpeed < 0) {
        pwmDriver.setPWM(motorA, 0, 0);
        pwmDriver.setPWM(motorB, 0, -currentRawSpeed);
    } else {
        pwmDriver.setPWM(motorA, 0, 4096);
        pwmDriver.setPWM(motorB, 0, 4096);
    }
}


Motor::Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB) {
    motorA = motorPortA;
    motorB = motorPortB;
    encA = encoderPortA;
    encB = encoderPortB;

    pinMode(encA, INPUT);
    pinMode(encB, INPUT);

    // Initialize member variables
    invertedMotor = false;
    invertedEncoder = false;
    CountsPerRevolution = 1.0f;
    currentRawSpeed = 0;
    currentCounts = 0;
    lastCounts = 0;
    current_rpm = 0.0f;
    currentVelocity = 0.0f;
    WheelDiameter = 0.0f;
}

Motor Motor::invertMotor(bool inverted){
    Motor::invertedMotor = inverted;
    return *this;
}

Motor Motor::invertEncoder(bool inverted){
    Motor::invertedEncoder = inverted;
    return *this;
}

Motor Motor::setCPR(float countsPerRevolution){
    Motor::CountsPerRevolution = countsPerRevolution;
    return *this;
}

int* Motor::setRawSpeed(int speed) {
    currentRawSpeed = speed;
    return &currentRawSpeed;
}

int* Motor::getSpeed() const {
    return (int*)&currentRawSpeed;
}

void Motor::updateCounts() {
    if (digitalRead(encA) == digitalRead(encB)) {
        currentCounts += invertedEncoder ? 1 : -1;  // Invert direction if flag is set
    } else {
        currentCounts += invertedEncoder ? -1 : 1;  // Invert direction if flag is set
    }
}

void Motor::printStatus() {
    Serial.print("Current Speed: \t");
    Serial.print(currentRawSpeed);
    Serial.print("\t|Current Counts: \t");
    Serial.print(currentCounts);
    Serial.print("\t|Current RPM: \t");
    Serial.println(current_rpm);
}

void Motor::coast(){
    pwmDriver.setPWM(motorA, 0, 0);
    pwmDriver.setPWM(motorB, 0, 0);
}

void Motor::brake(){
    pwmDriver.setPWM(motorA, 0, 4096);
    pwmDriver.setPWM(motorB, 0, 4096);
}


const int* Motor::getCounts() const {
    return &currentCounts;
}