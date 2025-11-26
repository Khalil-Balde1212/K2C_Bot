#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include <MotorInterface.h>
#include "inverse_kinematics.h"
#include "VelocityEstimator.h"
#include "SensorInterface.h"

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

    // Servos for arm and end effector
    Servo armServo1;  // First arm joint
    Servo armServo2;  // Second arm joint
    Servo endEffectorServo;  // Gripper or tool

    // PID Controllers
    PIDController leftDrivePID;
    PIDController rightDrivePID;
    PIDController leftPivotPID;
    PIDController rightPivotPID;
    
    // Velocity control PID controllers
    PIDController vxPID;
    PIDController vyPID;
    PIDController omegaPID;
    
    // Velocity estimator with Kalman filter
    VelocityEstimator velocityEstimator;

    // Motor configuration
    const float WHEEL_RADIUS = 0.015f;  // meters (30mm diameter wheels = 15mm radius)
    const float COUNTS_PER_REV_DRIVE = 2200.0f;
    const float COUNTS_PER_REV_PIVOT = 1440.0f;
    const float MAX_RPM = 100.0f;

    // State tracking
    float targetLeftRPM, targetRightRPM;
    float targetLeftAngle, targetRightAngle;
    float leftPivotOffset, rightPivotOffset;  // Zero calibration offsets
    bool initialized;
    unsigned long lastTime;
    
    // Velocity control state
    float targetVx, targetVy, targetOmega;  // Target velocities in robot frame (m/s, m/s, rad/s)
    float currentVx, currentVy, currentOmega;  // Current velocities (computed from encoders)
    float rawEncoderVx;  // Raw encoder velocity before Kalman filter (for slip detection debug)
    bool velocityControlEnabled;  // Flag to enable/disable velocity control mode
    unsigned long lastVelocityUpdate;  // Timestamp for velocity computation
    IMUInterface* imu;  // IMU reference for debug (moved here to fix initialization order)
    int lastLeftDriveCounts, lastRightDriveCounts;  // Previous encoder counts for velocity estimation

    // Conversion functions
    int rpmToPWM(float rpm);
    int angleToPWM(float angle_rad);
    
    // Velocity control helper functions
    void updateVelocityEstimate(unsigned long currentTime);
    void updateVelocityControl(unsigned long currentTime);

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
    void setVelocityControlGains(float kp_vx, float ki_vx, float kd_vx,
                                  float kp_vy, float ki_vy, float kd_vy,
                                  float kp_omega, float ki_omega, float kd_omega);

    // Control updates (call in main loop)
    void update(unsigned long currentTime);

    // Emergency stop
    void stop();
    void emergencyStop();

    // Velocity control
    void enableVelocityControl(bool enable);
    void setTargetVelocity(float vx, float vy, float omega);  // m/s, m/s, rad/s
    
    // IMU integration for Kalman filter
    void setIMUReference(IMUInterface* imuPtr);
    
    // Servo control for arm and end effector
    void setArmServosAngle(float angle);  // degrees
    void setEndEffectorAngle(float angle); // degrees
    float getArmServo1Angle() const;
    float getArmServo2Angle() const;
    float getEndEffectorAngle() const;
    
    // Slip detection
    bool isSlipping() const { return velocityEstimator.isSlipping(); }
    float getSlipAmount() const { return velocityEstimator.getSlipAmount(); }
    float getKalmanVelocity() const { return velocityEstimator.getVelocity(); }
    
    // Status
    bool isInitialized() const { return initialized; }
    bool isVelocityControlEnabled() const { return velocityControlEnabled; }
    float getLeftRPM() const;
    float getRightRPM() const;
    float getLeftAngle() const;  // radians
    float getRightAngle() const; // radians
    
    // Velocity getters
    float getCurrentVx() const { return currentVx; }
    float getCurrentVy() const { return currentVy; }
    float getCurrentOmega() const { return currentOmega; }
    float getTargetVx() const { return targetVx; }
    float getTargetVy() const { return targetVy; }
    float getTargetOmega() const { return targetOmega; }

    // Calibration
    void calibratePivotZero();  // Set current position as zero angle
    void resetPivotAngles();    // Reset target angles to current position

    // Debug output
    void printStatus();
};

#endif // ROBOT_CONTROLLER_H