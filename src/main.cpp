#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>
#include "inverse_kinematics/inverse_kinematics.h"
#include "forward_kinematics/fk.h"

// Motors: left/right drive, left/right steering
Motor leftMotor(12, 13, 2, 7);
Motor rightMotor(7, 6, 10, 9);
Motor leftPivot(15, 14, 11, 8);
Motor rightPivot(5, 4, 3, 4);

// Sensors
IMUInterface imu(20.0);

// Kinematics
FK odometry;

// Control gains
const float KP_HEADING = 2.5f;
const float KI_HEADING = 0.1f;
const float KD_HEADING = 0.05f;

// State
unsigned long lastTime = 0;
unsigned long lastProcessTime = 0;
float targetHeading = 0.0f;
float desiredSpeed = 1.0f; // m/s

// PID state
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;

// Store commanded velocities for FK
float lastCmdVx = 0.0f;
float lastCmdVy = 0.0f;
float lastCmdOmega = 0.0f;

void setup() {
    Serial.begin(115200);

    // Init motors
    Motor::begin();
    leftMotor.setCPR(1440.0f).invertMotor(true);
    rightMotor.setCPR(1440.0f);
    leftPivot.setCPR(2200.0f).invertEncoder(true);
    rightPivot.setCPR(2200.0f).invertEncoder(true);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(leftMotor.encA), 
                    []() { leftMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encA), 
                    []() { rightMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(leftPivot.encA), 
                    []() { leftPivot.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightPivot.encA), 
                    []() { rightPivot.updateCounts(); }, RISING);

    // Stop all
    leftMotor.setRawSpeed(0);
    rightMotor.setRawSpeed(0);
    leftPivot.setRawSpeed(0);
    rightPivot.setRawSpeed(0);

    // Init IMU
    if (!imu.begin()) {
        if (Serial) Serial.println("IMU init failed!");
        while (1);
    }
    imu.calibrateGyro(100);
    
    // Stabilize filter
    for (int i = 0; i < 100; i++) {
        imu.update();
        delay(10);
    }
    imu.calibrateOrientation();

    // Initialize FK at origin
    odometry.reset();

    if (Serial) Serial.println("Ready!");
}

// RPM to PWM
int rpmToPWM(float rpm) {
    float maxRPM = 100.0f;
    int pwm = (int)((rpm / maxRPM) * 4096.0f);
    return constrain(pwm, -4096, 4096);
}

// Steering angle to PWM
int angleToSteeringPWM(float angle_rad) {
    float angle_deg = angle_rad * 57.2958f;
    int pwm = (int)(angle_deg * 45.5f);
    return constrain(pwm, -4096, 4096);
}

// Execute motion using IK
void executeMotion(float vx, float vy, float omega) {
    IK ik(vx, vy, omega);
    
    // Store commanded velocities for FK
    lastCmdVx = vx;
    lastCmdVy = vy;
    lastCmdOmega = omega;
    
    // Set steering
    leftPivot.setRawSpeed(angleToSteeringPWM(ik.theta_left));
    rightPivot.setRawSpeed(angleToSteeringPWM(ik.theta_right));
    
    // Set drive speeds
    float leftSpeed = (ik.wheel_speeds[0] + ik.wheel_speeds[2]) / 2.0f;
    float rightSpeed = (ik.wheel_speeds[1] + ik.wheel_speeds[3]) / 2.0f;
    
    leftMotor.setRawSpeed(rpmToPWM(leftSpeed));
    rightMotor.setRawSpeed(rpmToPWM(rightSpeed));
}

// Normalize angle to [-180, 180]
float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void loop() {
    unsigned long currentTime = millis();
    
    // Update IMU at 20Hz
    if (currentTime - lastTime > 50) {
        lastTime = currentTime;
        imu.update();
    }
    
    // Control loop at 100Hz
    if (currentTime - lastProcessTime > 10) {
        float dt = (currentTime - lastProcessTime) / 1000.0f;
        lastProcessTime = currentTime;
        
        // Update odometry from commanded velocities
        odometry.updateFromVelocities(lastCmdVx, lastCmdVy, lastCmdOmega, dt);
        
        // Get heading from odometry
        float odomHeading = odometry.getHeading() * 57.2958f; // rad to deg
        
        // Fuse with IMU (trust IMU more since odometry is open-loop here)
        float imuHeading = imu.getYaw();
        float alpha = 0.2f; // Trust IMU 80%
        float fusedHeading = alpha * odomHeading + (1.0f - alpha) * imuHeading;
        
        // Compute heading error
        float headingError = normalizeAngle(targetHeading - fusedHeading);
        
        // PID control for heading
        headingErrorIntegral += headingError * dt;
        headingErrorIntegral = constrain(headingErrorIntegral, -10.0f, 10.0f);
        
        float headingErrorDerivative = (headingError - lastHeadingError) / dt;
        lastHeadingError = headingError;
        
        // Compute angular velocity command
        float omega = (KP_HEADING * headingError + 
                      KI_HEADING * headingErrorIntegral + 
                      KD_HEADING * headingErrorDerivative) * (PI / 180.0f);
        
        // Limit omega
        omega = constrain(omega, -1.0f, 1.0f);
        
        // Compute motion command
        float vx = 0.0f;
        float vy = desiredSpeed;
        
        // Execute motion
        executeMotion(vx, vy, omega);
        
        // Update motors
        leftMotor.update(lastProcessTime, currentTime);
        rightMotor.update(lastProcessTime, currentTime);
        leftPivot.update(lastProcessTime, currentTime);
        rightPivot.update(lastProcessTime, currentTime);
    }
    
    // Serial commands
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.startsWith("h")) {
            targetHeading = input.substring(1).toFloat();
            headingErrorIntegral = 0.0f;
            Serial.print("Target heading: ");
            Serial.println(targetHeading);
        } else if (input.startsWith("v")) {
            desiredSpeed = input.substring(1).toFloat();
            Serial.print("Speed: ");
            Serial.println(desiredSpeed);
        } else if (input == "stop") {
            desiredSpeed = 0.0f;
            executeMotion(0, 0, 0);
        } else if (input == "reset") {
            odometry.reset();
            imu.calibrateOrientation();
            targetHeading = 0.0f;
            headingErrorIntegral = 0.0f;
            Serial.println("Reset complete");
        }
    }
    
    // Debug output at 10Hz
    static unsigned long lastPrint = 0;
    if (Serial && currentTime - lastPrint > 100) {
        lastPrint = currentTime;
        Serial.print("Target: ");
        Serial.print(targetHeading, 1);
        Serial.print("° | IMU: ");
        Serial.print(imu.getYaw(), 1);
        Serial.print("° | Err: ");
        Serial.print(lastHeadingError, 1);
        Serial.print("° | Pos: (");
        Serial.print(odometry.getX(), 3);
        Serial.print(", ");
        Serial.print(odometry.getY(), 3);
        Serial.println(")");
    }
}