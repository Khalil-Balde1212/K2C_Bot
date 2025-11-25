#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>
#include "inverse_kinematics/inverse_kinematics.h"
#include "forward_kinematics/fk.h"

// Motors: left/right drive, left/right steering
Motor leftMotor(12, 13, 2, 7);
Motor leftPivot(15, 14, 11, 8);

Motor rightMotor(5, 4, 3, 4);
Motor rightPivot(7, 6, 10, 9); 


// Sensors
IMUInterface imu(20.0);
bool imuAvailable = false; // Track if IMU initialized successfully

// Kinematics
FK odometry;

// Control gains (reduced to keep omega well below 0.5 rad/s PIVOT threshold)
const float KP_HEADING = 2000.0f;
const float KI_HEADING = 0.0f;
const float KD_HEADING = 0.0f;

// State
unsigned long lastTime = 0;
unsigned long lastProcessTime = 0;
float targetHeading = 0.0f;
float desiredSpeed = 0.2f; // m/s (reduced from 5.0 to reasonable speed)
bool quietMode = false; // Enable debug output by default
bool directControlMode = false; // Direct motor control for testing

// PID state
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;

// Store commanded velocities for FK
float lastCmdVx = 0.0f;
float lastCmdVy = 0.0f;
float lastCmdOmega = 0.0f;

// Encoder tracking for odometry
int lastLeftCounts = 0;
int lastRightCounts = 0;
const float WHEEL_RADIUS = 0.00635f;  // meters (same as FK)
const float COUNTS_PER_REV = 1440.0f;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== K2C_Bot Starting ===");

    // Init motors
    Serial.println("Initializing motors...");
    Motor::begin();
    leftMotor.setCPR(2200.0f).invertMotor(true);
    rightMotor.setCPR(2200.0f).invertMotor(true);

    leftPivot.setCPR(1440.0f);
    rightPivot.setCPR(1440.0f);

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
    Serial.println("Motors initialized!");

    // Init IMU (FIXED: non-blocking, continues even if IMU fails)
    Serial.println("Initializing IMU...");
    if (!imu.begin()) {
        Serial.println("WARNING: IMU init failed! Continuing without IMU.");
        imuAvailable = false;
    } else {
        Serial.println("IMU initialized, calibrating...");
        imu.calibrateGyro(100);

        // Set magnetometer calibration (from magcal command)
        imu.setMagCalibration(-33.50f, 16.50f, 6.50f, 1.067f, 1.087f, 0.875f);

        // Stabilize filter
        for (int i = 0; i < 100; i++) {
            imu.update();
            delay(10);
        }
        imu.calibrateOrientation();
        imuAvailable = true;
        Serial.println("IMU calibrated!");
    }

    // Initialize FK at origin
    odometry.reset();

    // Initialize encoder tracking
    lastLeftCounts = *leftMotor.getCounts();
    lastRightCounts = *rightMotor.getCounts();

    Serial.println("=== Ready! ===");
    Serial.println("Commands: direct, auto, lm/rm/lp/rp <pwm>, stop, quiet, status, v<speed>, h<heading>");
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

    // Set drive speeds (right needs extra negation; left matches IK frame)
    float leftSpeed = (ik.wheel_speeds[0] + ik.wheel_speeds[2]) / 2.0f;
    float rightSpeed = (ik.wheel_speeds[1] + ik.wheel_speeds[3]) / 2.0f;

    leftMotor.setRawSpeed(rpmToPWM(leftSpeed));
    rightMotor.setRawSpeed(-rpmToPWM(rightSpeed));
}

// Normalize angle to [-180, 180]
float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void loop() {
    unsigned long currentTime = millis();

    // Update IMU at 20Hz (only if available)
    if (imuAvailable && currentTime - lastTime > 50) {
        lastTime = currentTime;
        imu.update();
    }

    // Control loop at 100Hz
    if (currentTime - lastProcessTime > 10) {
        float dt = (currentTime - lastProcessTime) / 1000.0f;
        lastProcessTime = currentTime;

        // Skip automatic control if in direct control mode
        if (!directControlMode) {
            // Get current encoder counts
            int leftCounts = *leftMotor.getCounts();
            int rightCounts = *rightMotor.getCounts();

            // Compute delta counts since last update
            int deltaLeft = leftCounts - lastLeftCounts;
            int deltaRight = rightCounts - lastRightCounts;
            lastLeftCounts = leftCounts;
            lastRightCounts = rightCounts;

            // Convert encoder counts to wheel displacement (meters)
            float distPerCount = (2.0f * PI * WHEEL_RADIUS) / COUNTS_PER_REV;
            float leftDist = deltaLeft * distPerCount;
            float rightDist = deltaRight * distPerCount;

            // Get steering angle from pivot encoder (approximate from left pivot)
            // CPR for pivot is 2200, convert counts to radians
            int pivotCounts = *leftPivot.getCounts();
            float theta_s = (pivotCounts / 2200.0f) * 2.0f * PI;

            // Build delta_s array [FL, FR, RL, RR] - using same displacement for front/rear
            float delta_s[4] = {leftDist, rightDist, leftDist, rightDist};

            // Update odometry from actual encoder measurements
            odometry.updateFromEncoders(delta_s, theta_s, dt);

            // Use IMU heading if available, otherwise assume no heading drift (0 degrees)
            float fusedHeading = imuAvailable ? imu.getYaw() : 0.0f;

            // Compute heading error
            float headingError = normalizeAngle(targetHeading - fusedHeading);

            // PID control for heading
            headingErrorIntegral += headingError * dt;
            headingErrorIntegral = constrain(headingErrorIntegral, -10.0f, 10.0f);

            float headingErrorDerivative = (headingError - lastHeadingError) / dt;
            lastHeadingError = headingError;

            // Compute angular velocity command
            // Positive error (robot left of target) needs negative omega (turn right/CW)
            // So omega = -K * error gives correct sign
            float omega = -(KP_HEADING * headingError +
                            KI_HEADING * headingErrorIntegral +
                            KD_HEADING * headingErrorDerivative) * (PI / 180.0f);

            // Limit omega to stay below PIVOT threshold (0.5 rad/s)
            omega = constrain(omega, -0.4f, 0.4f);

            // Move forward at desired speed while correcting heading
            float vx = desiredSpeed;
            float vy = 0.0f;

            // Execute motion
            executeMotion(vx, vy, omega);
        }

        // Always update motors
        leftMotor.update(lastProcessTime, currentTime);
        rightMotor.update(lastProcessTime, currentTime);
        leftPivot.update(lastProcessTime, currentTime);
        rightPivot.update(lastProcessTime, currentTime);
    }

    // Serial commands
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        switch (input.charAt(0)) {
            case 'h': {
            targetHeading = input.substring(1).toFloat();
            headingErrorIntegral = 0.0f;
            Serial.print("Target heading: ");
            Serial.println(targetHeading);
            break;
            }
            case 'v': {
            desiredSpeed = input.substring(1).toFloat();
            Serial.print("Speed: ");
            Serial.println(desiredSpeed);
            break;
            }
            case 'l': {
                desiredSpeed = input.substring(1).toFloat();
                leftMotor.setRawSpeed(desiredSpeed*4095);
                Serial.print("Left Motor Speed: ");
                Serial.println(desiredSpeed);
                break;
            }
            case 'r': {
                desiredSpeed = input.substring(1).toFloat();
                rightMotor.setRawSpeed(desiredSpeed*4095);
                Serial.print("Right Motor Speed: ");
                Serial.println(desiredSpeed);
                break;
            }
            case 'p': {
                desiredSpeed = input.substring(1).toFloat();
                leftPivot.setRawSpeed(desiredSpeed*4095);
                Serial.print("Pivot Speed: ");
                Serial.println(desiredSpeed);
                break;
            }
            case 'q': {
                desiredSpeed = input.substring(1).toFloat();
                rightPivot.setRawSpeed(desiredSpeed*4095);
                Serial.print("Right Pivot Speed: ");
                Serial.println(desiredSpeed);
                break;
            }
            default: {
            if (input == "stop") {
                desiredSpeed = 0.0f;
                executeMotion(0, 0, 0);
            } else if (input == "reset") {
                odometry.reset();
                imu.calibrateOrientation();
                targetHeading = 0.0f;
                headingErrorIntegral = 0.0f;
                lastLeftCounts = *leftMotor.getCounts();
                lastRightCounts = *rightMotor.getCounts();
                Serial.println("Reset complete");
            } else if (input == "magcal") {
                // Stop motors during calibration
                executeMotion(0, 0, 0);
                imu.calibrateMagnetometer(15);  // 15 seconds to rotate robot
                imu.calibrateOrientation();
                Serial.println("Copy the calibration values above to setMagCalibration() in setup()");
            }
            break;
            }
        }
    }

    // Debug output at 10Hz (only if not in quiet mode)
    static unsigned long lastPrint = 0;
    if (!quietMode && Serial && currentTime - lastPrint > 100) {
        lastPrint = currentTime;
        // Serial.print("Target: ");
        // Serial.print(targetHeading, 1);
        // Serial.print("° | IMU: ");
        // Serial.print(imu.getYaw(), 1);
        // Serial.print("° | Err: ");
        // Serial.print(lastHeadingError, 1);
        // Serial.print("° | Pos: (");
        // Serial.print(odometry.getX(), 3);
        // Serial.print(", ");
        // Serial.print(odometry.getY(), 3);
        // Serial.print(") | Mag: (");
        // Serial.print(imu.getMx(), 1);
        // Serial.print(", ");
        // Serial.print(imu.getMy(), 1);
        // Serial.print(", ");
        // Serial.print(imu.getMz(), 1);
        // Serial.println(")");
    
        Serial.print("Left Counts:\t");
        Serial.print(*leftMotor.getCounts());
        Serial.print(" | Right Counts:\t");
        Serial.print(*rightMotor.getCounts());
        Serial.print(" | LeftP Counts:\t");
        Serial.print(*leftPivot.getCounts());
        Serial.print(" | RightP Counts:\t");
        Serial.println(*rightPivot.getCounts());
    }
}
