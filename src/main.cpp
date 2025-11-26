#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>
#include <RobotController.h>
#include "inverse_kinematics.h"
#include "fk.h"

// Motors: left/right drive, left/right steering
Motor leftMotor(12, 13, 2, 7);
Motor leftPivot(15, 14, 11, 8);

Motor rightMotor(5, 4, 3, 4);
Motor rightPivot(7, 6, 10, 9);

// Robot Controller
RobotController robot(&leftMotor, &rightMotor, &leftPivot, &rightPivot);

// Sensors
IMUInterface imu(20.0);
bool imuAvailable = false; // Track if IMU initialized successfully

TOF::TOFSensors tofSensors;

// Kinematics
FK odometry;

// Control gains (reduced to keep omega well below 0.5 rad/s PIVOT threshold)
const float KP_HEADING = 2000.0f;
const float KI_HEADING = 0.0f;
const float KD_HEADING = 0.0f;

// State
unsigned long lastTime = 0;
float targetHeading = 0.0f;
float desiredSpeed = 0.2f; // m/s (reduced from 5.0 to reasonable speed)
bool quietMode = false;    // Enable debug output by default

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
const float WHEEL_RADIUS = 0.00635f; // meters (same as FK)
const float COUNTS_PER_REV = 1440.0f;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== K2C_Bot Starting ===");

    // Init motors
    Serial.println("Initializing motors...");
    Motor::begin();

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(leftMotor.encA), []()
                    { leftMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encA), []()
                    { rightMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(leftPivot.encA), []()
                    { leftPivot.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightPivot.encA), []()
                    { rightPivot.updateCounts(); }, RISING);

    // Initialize Robot Controller
    Serial.println("Initializing robot controller...");
    if (!robot.begin())
    {
        Serial.println("ERROR: Robot controller initialization failed!");
        while (1)
            ; // Halt if controller fails
    }

    Serial.println("Motors and controller initialized!");

    // Init IMU (FIXED: non-blocking, continues even if IMU fails)
    Serial.println("Initializing IMU...");
    if (!imu.begin())
    {
        Serial.println("WARNING: IMU init failed! Continuing without IMU.");
        imuAvailable = false;
    }
    else
    {
        Serial.println("IMU initialized, calibrating...");
        imu.calibrateGyro(100);

        // Set magnetometer calibration (from magcal command)
        imu.setMagCalibration(-33.50f, 16.50f, 6.50f, 1.067f, 1.087f, 0.875f);

        // Stabilize filter
        for (int i = 0; i < 100; i++)
        {
            imu.update();
            delay(10);
        }
        imu.calibrateOrientation();

        // Enable continuous bias estimation for drift reduction
        imu.enableBiasEstimation(true);

        imuAvailable = true;
        Serial.println("IMU calibrated!");
        
        // Connect IMU to robot controller for Kalman filter
        robot.setIMUReference(&imu);
    }

    // Initialize FK at origin
    odometry.reset();

    // Initialize encoder tracking
    lastLeftCounts = *leftMotor.getCounts();
    lastRightCounts = *rightMotor.getCounts();

    // TOF Sensors
    Serial.println("Initializing TOF sensors...");
    tofSensors.initialize(
        new int[TOF::SENSOR_COUNT]{5, 13},          // XSHUT pins 5, 6, 12, 13
        new uint8_t[TOF::SENSOR_COUNT]{0x30, 0x31}, // I2C addresses
        A4, A5                                      // SDA, SCL
    );
    if (!tofSensors.begin())
    {
        Serial.println("WARNING: TOF sensors init failed! Continuing without TOF sensors.");
    }
    else
    {
        Serial.println("TOF sensors initialized!");
        // Set calibration offsets
        tofSensors.setSensorOffset(0, -7.0f); // Left sensor offset
        tofSensors.setSensorOffset(1, 11.0f); // Right sensor offset
    }


    Serial.println("=== Ready! ===");
    Serial.println("Commands: v<speed>, h<heading>, l<rpm>, r<rpm>, p<angle_deg>, q<angle_deg>, al<angle>, ar<angle>, stop, reset, magcal, pivotzero, pivotreset, status");
    Serial.println("Velocity Control: velctl on/off, veltarget <vx> <vy> <omega>, velgains <kp> <ki> <kd>");
}

// Execute motion using Robot Controller
void executeMotion(float vx, float vy, float omega)
{
    robot.setVelocity(vx, vy, omega);

    // Store commanded velocities for FK
    lastCmdVx = vx;
    lastCmdVy = vy;
    lastCmdOmega = omega;
}

// Normalize angle to [-180, 180]
float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

void loop()
{
    unsigned long currentTime = millis();

    if (imuAvailable && currentTime - lastTime > 10)
    {
        imu.update();
    }

    // Serial commands
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Check for velocity control commands first (before switch statement)
        // to avoid conflict with 'v' speed command
        if (input.startsWith("velctl"))
        {
            String command = input.substring(7);
            command.trim();
            if (command == "on")
            {
                robot.enableVelocityControl(true);
            }
            else if (command == "off")
            {
                robot.enableVelocityControl(false);
            }
            else
            {
                Serial.println("Usage: velctl on/off");
            }
        }
        else if (input.startsWith("veltarget"))
        {
            // Parse: veltarget <vx> <vy> <omega>
            String params = input.substring(10);
            params.trim();
            
            int firstSpace = params.indexOf(' ');
            int secondSpace = params.indexOf(' ', firstSpace + 1);
            
            if (firstSpace > 0 && secondSpace > 0)
            {
                float vx = params.substring(0, firstSpace).toFloat();
                float vy = params.substring(firstSpace + 1, secondSpace).toFloat();
                float omega = params.substring(secondSpace + 1).toFloat();
                
                robot.setTargetVelocity(vx, vy, omega);
                Serial.print("Target velocity set: vx=");
                Serial.print(vx, 3);
                Serial.print(" m/s, vy=");
                Serial.print(vy, 3);
                Serial.print(" m/s, omega=");
                Serial.print(omega, 3);
                Serial.println(" rad/s");
            }
            else
            {
                Serial.println("Usage: veltarget <vx> <vy> <omega>");
            }
        }
        else if (input.startsWith("velgains"))
        {
            // Parse: velgains <kp> <ki> <kd> (applies to all three controllers)
            String params = input.substring(9);
            params.trim();
            
            int firstSpace = params.indexOf(' ');
            int secondSpace = params.indexOf(' ', firstSpace + 1);
            
            if (firstSpace > 0 && secondSpace > 0)
            {
                float kp = params.substring(0, firstSpace).toFloat();
                float ki = params.substring(firstSpace + 1, secondSpace).toFloat();
                float kd = params.substring(secondSpace + 1).toFloat();
                
                robot.setVelocityControlGains(kp, ki, kd, kp, ki, kd, kp, ki, kd);
                Serial.print("Velocity PID gains set: kp=");
                Serial.print(kp, 2);
                Serial.print(", ki=");
                Serial.print(ki, 2);
                Serial.print(", kd=");
                Serial.println(kd, 2);
            }
            else
            {
                Serial.println("Usage: velgains <kp> <ki> <kd>");
            }
        }
        else
        {
            // Standard character-based commands
            switch (input.charAt(0))
            {
        case 'h':
        {
            targetHeading = input.substring(1).toFloat();
            headingErrorIntegral = 0.0f;
            Serial.print("Target heading: ");
            Serial.println(targetHeading);
            break;
        }
        case 'v':
        {
            desiredSpeed = input.substring(1).toFloat();
            Serial.print("Speed: ");
            Serial.println(desiredSpeed);
            break;
        }
        case 'a':
        {
            char side = input.charAt(1);
            String angleStr = input.substring(2);
            float angle = angleStr.toFloat();
            
            if (side == 'l')
            {
                Serial.print("Setting left pivot angle: ");
                Serial.println(angle);
                robot.setSteeringAngles(angle * PI / 180.0f, robot.getRightAngle());
            }
            else if (side == 'r')
            {
                Serial.print("Setting right pivot angle: ");
                Serial.println(angle);
                robot.setSteeringAngles(robot.getLeftAngle(), angle * PI / 180.0f);
            } else {
                angle = input.substring(1).toFloat();
                Serial.println("Setting pivot angle: " + String(angle));
                robot.setSteeringAngles(angle * PI / 180.0f, angle * PI / 180.0f);
            }
            break;
        }
        case 's':
        {
            char side = input.charAt(1);
            float speed = input.substring(2).toFloat();
            if (side == 'l')
            {
                Serial.print("Setting left motor speed: ");
                Serial.println(speed);
                robot.setWheelSpeeds(speed, robot.getRightRPM());
            }
            else if (side == 'r')
            {
                Serial.print("Setting right motor speed: ");
                Serial.println(speed);
                robot.setWheelSpeeds(robot.getLeftRPM(), speed);
            }
            break;
        }
        default:
        {
            if (input == "stop")
            {
                desiredSpeed = 0.0f;
                robot.stop();
            }
            else if (input == "reset")
            {
                odometry.reset();
                imu.calibrateOrientation();
                imu.resetBiasEstimation();
                robot.stop();
                targetHeading = 0.0f;
                headingErrorIntegral = 0.0f;
                lastLeftCounts = *leftMotor.getCounts();
                lastRightCounts = *rightMotor.getCounts();
                Serial.println("Reset complete");
            }
            else if (input == "magcal")
            {
                // Stop motors during calibration
                robot.emergencyStop();
                imu.calibrateMagnetometer(15); // 15 seconds to rotate robot
                imu.calibrateOrientation();
                Serial.println("Copy the calibration values above to setMagCalibration() in setup()");
            }
            else if (input == "pivotzero")
            {
                robot.calibratePivotZero();
            }
            else if (input == "pivotreset")
            {
                robot.resetPivotAngles();
            }
            else if (input == "quiet")
            {
                quietMode = !quietMode;
                Serial.print("Quiet mode: ");
                Serial.println(quietMode ? "ON" : "OFF");
            }
            break;
        }
        }
        }
    }

    // Debug output at 10Hz (only if not in quiet mode)
    static unsigned long lastPrint = 0;
    if (!quietMode && Serial && currentTime - lastPrint > 1000)
    {
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

        // Robot controller status
        robot.printStatus();

        // Serial.print("leftPivot Counts:\t");
        // Serial.print(*leftPivot.getCounts());
        // Serial.print(" | rightPivot Counts:\t");
        // Serial.print(*rightPivot.getCounts());
        // Serial.print(" | leftMotor Counts:\t");
        // Serial.print(*leftMotor.getCounts());
        // Serial.print(" | rightMotor Counts:\t");
        // Serial.print(*rightMotor.getCounts());
        // Serial.print(" | leftMotor Speed:\t");
        // Serial.print(*leftMotor.getSpeed());
        // Serial.print(" | rightMotor Speed:\t");
        // Serial.print(*rightMotor.getSpeed());
        // Serial.print(" | leftPivot Speed:\t");
        // Serial.print(*leftPivot.getSpeed());
        // Serial.print(" | rightPivot Speed:\t");
        // Serial.print(*rightPivot.getSpeed());
        // Serial.println();
        // Update last time for next iteration
    }

    robot.update(currentTime);

    lastTime = currentTime;
}