#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Motor
{
public:

    Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB);

    Motor invertMotor(bool inverted = true);
    Motor invertEncoder(bool inverted = true);
    Motor setCPR(float countsPerRevolution);

    int* setRawSpeed(int speed);
    int* getSpeed() const;
    void coast();
    void brake();

    void setVelocity(double velocity);

    void updateCounts();

    const int* getCounts() const;
    const float* currentRPM() const;

    void update(unsigned long currentTime);

    // Static members shared by all Motor objects
    static Adafruit_PWMServoDriver pwmDriver;
    static void begin();


    void printStatus();
    int motorA, motorB;
    int encA, encB;

private:
    int deadband = 150; // Speed deadband
    float CountsPerRevolution;
    int currentRawSpeed;
    int currentCounts, lastCounts;
    float current_rpm;
    float currentVelocity;

    bool invertedMotor;
    bool invertedEncoder;

    int speed;
    unsigned long lastUpdateTime;

    float WheelDiameter; // inche
};

#endif  