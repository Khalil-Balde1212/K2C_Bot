#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include <MotorInterface.h>
#include "inverse_kinematics/inverse_kinematics.h"

// PID Controller class
class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float outputMin, outputMax;
    unsigned long lastTime;

public:
    PIDController(float p = 0.0f, float i = 0.0f, float d = 0.0f,
                  float min = -4096.0f, float max = 4096.0f);

    void setGains(float p, float i, float d);
    void setLimits(float min, float max);
    void reset();

    float compute(float setpoint, float measurement, unsigned long currentTime);
    float getIntegral() const { return integral; }
};

// Robot Controller class
class RobotController {
private:
    // Motors
    Motor* leftMotor;
    Motor* rightMotor;
    Motor* leftPivot;
    Motor* rightPivot;

    // PID Controllers
    PIDController leftDrivePID;
    PIDController rightDrivePID;
    PIDController leftPivotPID;
    PIDController rightPivotPID;

    // Motor configuration
    const float WHEEL_RADIUS = 0.00635f;  // meters
    const float COUNTS_PER_REV_DRIVE = 2200.0f;
    const float COUNTS_PER_REV_PIVOT = 1440.0f;
    const float MAX_RPM = 100.0f;

    // State tracking
    float targetLeftRPM, targetRightRPM;
    float targetLeftAngle, targetRightAngle;
    bool initialized;

    // Conversion functions
    int rpmToPWM(float rpm);
    int angleToPWM(float angle_rad);

public:
    RobotController(Motor* lm, Motor* rm, Motor* lp, Motor* rp);

    // Initialization
    bool begin();

    // High-level motion control
    void setVelocity(float vx, float vy, float omega);  // m/s, m/s, rad/s
    void setWheelSpeeds(float leftRPM, float rightRPM);  // Direct RPM control
    void setSteeringAngles(float leftAngle, float rightAngle);  // radians

    // PID tuning
    void setDrivePIDGains(float kp, float ki, float kd);
    void setPivotPIDGains(float kp, float ki, float kd);

    // Control updates (call in main loop)
    void update(unsigned long currentTime);

    // Emergency stop
    void stop();
    void emergencyStop();

    // Status
    bool isInitialized() const { return initialized; }
    float getLeftRPM() const;
    float getRightRPM() const;
    float getLeftAngle() const;  // radians
    float getRightAngle() const; // radians

    // Debug
    void printStatus();
};

#endif // ROBOT_CONTROLLER_H