#include "MotorInterface.h"

#ifndef PWM_DRIVER
Adafruit_PWMServoDriver Motor::pwmDriver = Adafruit_PWMServoDriver();
#endif

void Motor::begin()
{
    pwmDriver.begin();
    pwmDriver.setPWMFreq(50); // This is the maximum PWM frequency
    Serial.println("Motor PWM Driver Initialized");
}

void Motor::update(unsigned long currentTime)
{
    unsigned long deltaTime = currentTime - lastUpdateTime; // in milliseconds

    // Calculate RPM
    int deltaCounts = currentCounts - lastCounts;
    current_rpm = (deltaCounts / CountsPerRevolution) / deltaTime; // rotations per ms
    current_rpm *= 1000;                                           // rotations per second
    current_rpm *= 60;                                             // rotations per minute
    lastCounts = currentCounts;

    // Apply inversion if set
    if (invertedMotor)
    {
        speed = -currentRawSpeed;
    }
    else
    {
        speed = currentRawSpeed;
    }
    speed = constrain(speed, -4095, 4095);

    // Set PWM values based on direction
    if (speed > deadzone)
    {
        pwmDriver.setPWM(motorA, 0, speed);
        pwmDriver.setPWM(motorB, 0, 0);
    }
    else if (speed < -deadzone)
    {
        pwmDriver.setPWM(motorA, 0, 0);
        pwmDriver.setPWM(motorB, 0, -speed);
    }
    else
    {
        pwmDriver.setPWM(motorA, 0, 4095);
        pwmDriver.setPWM(motorB, 0, 4095);
    }
    this->lastUpdateTime = millis();
}

Motor::Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB)
{
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
    speed = 0;
    lastUpdateTime = millis();
}

Motor Motor::invertMotor(bool inverted)
{
    this->invertedMotor = inverted;
    return *this;
}

Motor Motor::invertEncoder(bool inverted)
{
    this->invertedEncoder = inverted;
    return *this;
}

Motor Motor::setCPR(float countsPerRevolution)
{
    this->CountsPerRevolution = countsPerRevolution;
    return *this;
}

int *Motor::setRawSpeed(int speed)
{
    currentRawSpeed = speed;
    return &currentRawSpeed;
}

int *Motor::getSpeed() const
{
    return (int *)&currentRawSpeed;
}

void Motor::updateCounts()
{
    if (digitalRead(encA) == digitalRead(encB))
    {
        currentCounts += invertedEncoder ? 1 : -1; // Invert direction if flag is set
    }
    else
    {
        currentCounts += invertedEncoder ? -1 : 1; // Invert direction if flag is set
    }
}

void Motor::printStatus()
{
    Serial.print("Current Speed: \t");
    Serial.print(currentRawSpeed);
    Serial.print("\t|Current Counts: \t");
    Serial.print(currentCounts);
    Serial.print("\t|Current RPM: \t");
    Serial.println(current_rpm);
}

void Motor::coast()
{
    pwmDriver.setPWM(motorA, 0, 0);
    pwmDriver.setPWM(motorB, 0, 0);
}

void Motor::brake()
{
    pwmDriver.setPWM(motorA, 0, 4096);
    pwmDriver.setPWM(motorB, 0, 4096);
}

const int *Motor::getCounts() const
{
    return &currentCounts;
}

const float *Motor::currentRPM() const
{
    return &current_rpm;
}

// Servo implementation
Servo::Servo(int channel, int minPulse, int maxPulse)
    : channel(channel), minPulseUs(minPulse), maxPulseUs(maxPulse), currentAngle(90.0f), currentPulse(1500)
{
}

void Servo::setAngle(float angle)
{
    currentAngle = constrain(angle, 0.0f, 180.0f);
    currentPulse = map(currentAngle, 0, 180, minPulseUs, maxPulseUs);
    int pwmValue = map(currentPulse, 0, 20000, 0, 4095); // 20ms period at 50Hz
    Motor::pwmDriver.setPWM(channel, 0, pwmValue);
}

float Servo::getAngle() const
{
    return currentAngle;
}

void Servo::setPulse(int pulse)
{
    currentPulse = constrain(pulse, minPulseUs, maxPulseUs);
    currentAngle = map(currentPulse, minPulseUs, maxPulseUs, 0, 180);
    int pwmValue = map(currentPulse, 0, 20000, 0, 4095);
    Motor::pwmDriver.setPWM(channel, 0, pwmValue);
}

int Servo::getPulse() const
{
    return currentPulse;
}
