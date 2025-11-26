#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>
#include <RobotController.h>

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

// Control gains for heading control
const float KP_HEADING = 1000.0f;
const float KI_HEADING = 0.00f;
const float KD_HEADING = 0.0f;

// State
unsigned long lastTime = 0;
float targetHeading = 0.0f;
float desiredSpeed = 0.2f; // m/s (reduced from 5.0 to reasonable speed)
bool quietMode = false;    // Enable debug output by default

// PID state
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;

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
    }

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
    Serial.println("Note: 'v' command maintains current heading using IMU");
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

// Process a serial command string
void processCommand(String input)
{
    input.trim();

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

        // Set target heading to current heading for straight line movement
        if (imuAvailable) {
            targetHeading = imu.getYaw();
            headingErrorIntegral = 0.0f; // Reset integral when starting new movement
            Serial.print("Target heading set to current: ");
            Serial.println(targetHeading);
        }

        // Don't set wheel speeds here - let the heading control in loop() handle it
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
        }
        else
        {
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
            targetHeading = 0.0f; // Reset target heading when stopping
            headingErrorIntegral = 0.0f;
            robot.stop();
        }
        else if (input == "reset")
        {
            imu.calibrateOrientation();
            imu.resetBiasEstimation();
            robot.stop();
            targetHeading = 0.0f;
            desiredSpeed = 0.0f;
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
        break;
    }
    }
}

// Simulate sending a serial command to the robot
void sendSerialCommand(String command)
{
    Serial.print("Simulating command: ");
    Serial.println(command);
    processCommand(command);
}




void wallFollow()
{
    static unsigned long lastWallFollowTime = 0;
    unsigned long currentTime = millis();
    
    // Only update wall following at 10Hz (every 100ms) to prevent oscillation
    if (currentTime - lastWallFollowTime < 100) {
        return;
    }
    lastWallFollowTime = currentTime;

    float setpoint = 10.0f;
    float sensorReading = tofSensors.getFilteredDistance(1);
    
    // Check if sensor reading is valid (not 0 or invalid)
    if (sensorReading <= 0 || sensorReading > 2000) {
        Serial.println("Wall follow: Invalid sensor reading");
        return; // Skip if sensor reading is invalid
    }
    
    float error = sensorReading - setpoint; // left sensor

    // Simple P controller
    float Kp = 10.0f; // Reduced gain for stability
    float correction = Kp * -error;

    // Constrain to reasonable steering angles
    correction = constrain(correction, -45.0f, 45.0f);

    // Debug output
    Serial.print("Wall follow - Sensor: ");
    Serial.print(sensorReading, 1);
    Serial.print("mm, Error: ");
    Serial.print(error, 1);
    Serial.print(", Correction: ");
    Serial.print(correction, 1);
    Serial.println("°");

    // Set steering angles directly instead of using serial command
    robot.setSteeringAngles(correction * PI / 180.0f, correction * PI / 180.0f);
    processCommand("v2000"); // Maintain a speed of 0.2 m/s
}

void corridorFollow()
{
    float error = tofSensors.getFilteredDistance(0) - tofSensors.getFilteredDistance(1); // left and right error
}

void loop()
{
    unsigned long currentTime = millis();
    wallFollow();

    if (imuAvailable && currentTime - lastTime > 10)
    {
        imu.update();
        
        // Heading control - adjust wheel speeds to maintain target heading
        if (targetHeading != 0.0f || desiredSpeed != 0.0f) {
            float currentHeading = imu.getYaw();
            float headingError = normalizeAngle(targetHeading - currentHeading);
            
            // PID control for heading
            headingErrorIntegral += headingError * 0.01f; // dt = 10ms
            headingErrorIntegral = constrain(headingErrorIntegral, -50.0f, 50.0f); // Anti-windup
            
            float headingDerivative = (headingError - lastHeadingError) / 0.01f;
            lastHeadingError = headingError;
            
            float headingCorrection = KP_HEADING * headingError + 
                                    KI_HEADING * headingErrorIntegral + 
                                    KD_HEADING * headingDerivative;
            
            // Apply heading correction to wheel speeds
            float leftSpeed = desiredSpeed - headingCorrection;
            float rightSpeed = desiredSpeed + headingCorrection;
            
            // Constrain speeds to reasonable limits
            leftSpeed = constrain(leftSpeed, -1000.0f, 1000.0f);
            rightSpeed = constrain(rightSpeed, -1000.0f, 1000.0f);
            
            robot.setWheelSpeeds(leftSpeed, rightSpeed);
        }
    }

    // Serial commands
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        processCommand(input);
    }

    // Debug output at 10Hz (only if not in quiet mode)
    static unsigned long lastPrint = 0;
    if (!quietMode && Serial && currentTime - lastPrint > 1000)
    {
        lastPrint = currentTime;
        if (imuAvailable)
        {
            Serial.print("IMU Yaw:\t");
            Serial.print(imu.getYaw(), 2);
            Serial.print(" | Target:\t");
            Serial.print(targetHeading, 2);
            Serial.print(" | Speed:\t");
            Serial.print(desiredSpeed, 1);
            Serial.println();
        }

        // Robot controller status
        // robot.printStatus();

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

        // Serial.print("TOF0 Distance: " + String(tofSensors.getFilteredDistance(0)) + " mm\t");
        // Serial.print(" | TOF1 Distance: " + String(tofSensors.getFilteredDistance(1)) + " mm");
        Serial.println();
        // Update last time for next iteration
    }

    robot.update(currentTime);

    lastTime = currentTime;
}