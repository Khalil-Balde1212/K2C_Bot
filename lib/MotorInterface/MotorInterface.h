#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Motor
{
public:

    Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB);

    Motor isInverted(bool inverted = true);
    int* setRawSpeed(int speed);
    int getSpeed() const;
    void stop();

    void setVelocity(double velocity);

    void updateCounts();

    const int* getCounts() const;
    const float* currentRPM() const;

    void update(unsigned long lastTime, unsigned long currentTime);

    // Static members shared by all Motor objects
    static Adafruit_PWMServoDriver pwmDriver;
    static void begin();


    void printStatus();
    int motorA, motorB;
    int encA, encB;
    float CountsPerRevolution;

private:

    int currentRawSpeed;
    int currentCounts, lastCounts;
    float current_rpm;
    float currentVelocity;

    bool inverted;

    float WheelDiameter; // inche
};

#endif  