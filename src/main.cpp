#include "main.h"
#include "inverse_kinematics.h"
#include "fk.h"

// Motors: left/right drive, left/right steering
Motor leftMotor(12, 13, 2, 7);
Motor leftPivot(15, 14, 11, 8);

Motor rightMotor(5, 4, 3, 4);
Motor rightPivot(7, 6, 10, 9);

// Robot Controller
RobotController robot(&leftMotor, &rightMotor, &leftPivot, &rightPivot);

// Sensors & kinematics
IMUInterface imu(20.0);
bool imuAvailable = false;
TOF::TOFSensors tofSensors;
FK odometry;

// Control gains
const float KP_HEADING = 2000.0f;
const float KI_HEADING = 0.0f;
const float KD_HEADING = 0.0f;

// State
unsigned long lastTime = 0;
float targetHeading = 0.0f;
float desiredSpeed = 0.2f; // m/s
bool quietMode = false;

// PID state
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;

// Encoder tracking
int lastLeftCounts = 0;
int lastRightCounts = 0;

// Store commanded velocities for FK
float lastCmdVx = 0.0f;
float lastCmdVy = 0.0f;
float lastCmdOmega = 0.0f;

// Forward declarations for demo functions
void startMainDemo();
void stopMainDemo();
void updateMainDemo(unsigned long currentTime);

// Setup initializes hardware, controller, and sensors
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== K2C_Bot Starting ===");

    // Init motors and attach interrupts
    Motor::begin();
    attachInterrupt(digitalPinToInterrupt(leftMotor.encA), []() { leftMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encA), []() { rightMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(leftPivot.encA), []() { leftPivot.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightPivot.encA), []() { rightPivot.updateCounts(); }, RISING);

    // Initialize RobotController
    if (!robot.begin())
    {
        Serial.println("ERROR: Robot controller initialization failed!");
        while (1) ;
    }

    // Initialize IMU
    Serial.println("Initializing IMU...");
    if (!imu.begin())
    {
        Serial.println("WARNING: IMU init failed! Continuing without IMU.");
        imuAvailable = false;
    }
    else
    {
        imu.calibrateGyro(100);
        imu.calibrateOrientation();
        imuAvailable = true;
        robot.setIMUReference(&imu);
    }

    // Initialize odometry and encoder tracking
    odometry.reset();
    lastLeftCounts = *leftMotor.getCounts();
    lastRightCounts = *rightMotor.getCounts();

    Serial.println("=== Ready! Use 'demo' or 'demo2' commands to run demos ===");
}

// Main loop calls out to the helper modules
void loop()
{
    unsigned long currentTime = millis();

    if (imuAvailable && currentTime - lastTime > 10)
    {
        imu.update();
    }

    // Handle serial commands in a separate file
    serialCommands();

    // Update sensor-based demo (if active)
    updateMainDemo(currentTime);

    // Debug prints in a separate file
    debugPrints(currentTime);

    // Update controller and motors
    robot.update(currentTime);

    lastTime = currentTime;
}

// Main demo functions: start/stop/update (sensor-based demo)
bool mainDemoEnabled = false;
float mainDemoStartX = 0.0f;
float mainDemoStartY = 0.0f;
float mainDemoStartYaw = 0.0f;
unsigned long lastOdometryUpdate = 0;

const float DEMO_FORWARD_DISTANCE = 0.3f;
const float DEMO_LATERAL_DISTANCE = 0.25f;
const float DEMO_TURN_ANGLE = 90.0f;

void startMainDemo()
{
    if (!robot.isInitialized()) return;
    mainDemoEnabled = true;
    mainDemoStartX = odometry.getX();
    mainDemoStartY = odometry.getY();
    mainDemoStartYaw = imuAvailable ? imu.getYaw() : odometry.getHeading() * 180.0f / PI;
    lastOdometryUpdate = millis();
    robot.enableVelocityControl(true);
    Serial.println("Main demo started");
}

void stopMainDemo()
{
    mainDemoEnabled = false;
    robot.setTargetVelocity(0.0f, 0.0f, 0.0f);
    Serial.println("Main demo stopped.");
}

void restartMainDemo()
{
    stopMainDemo();
    startMainDemo();
    Serial.println("Main demo restarted.");
}

void updateMainDemo(unsigned long currentTime)
{
    if (!mainDemoEnabled) return;

    float dt = (currentTime - lastOdometryUpdate) / 1000.0f;
    if (dt > 0.0f)
    {
        odometry.updateFromVelocities(robot.getCurrentVx(), robot.getCurrentVy(), robot.getCurrentOmega(), dt);
        lastOdometryUpdate = currentTime;
    }

    static int state = 0; // 0=FWD1,1=TURN,2=FWD2,3=CRAB,4=RIGHT,5=DONE
    float dx = odometry.getX() - mainDemoStartX;
    float dy = odometry.getY() - mainDemoStartY;

    auto angleDiff = [](float a, float b) {
        float d = a - b;
        while (d > 180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return fabs(d);
    };

    switch (state)
    {
        case 0: // move forward
            robot.setTargetVelocity(desiredSpeed, 0.0f, 0.0f);
            if (sqrt(dx*dx + dy*dy) >= DEMO_FORWARD_DISTANCE)
            {
                state = 1;
                mainDemoStartYaw = imuAvailable ? imu.getYaw() : odometry.getHeading() * 180.0f / PI;
                Serial.println("Demo: TURN_ON_SPOT");
            }
            break;
        case 1: // turn
        {
            float currentYaw = imuAvailable ? imu.getYaw() : odometry.getHeading() * 180.0f / PI;
            if (angleDiff(currentYaw, mainDemoStartYaw) >= DEMO_TURN_ANGLE)
            {
                state = 2;
                mainDemoStartX = odometry.getX();
                mainDemoStartY = odometry.getY();
                Serial.println("Demo: MOVE_FORWARD2");
            }
            else
            {
                float targetOmega = (PI/2.0f) / 1.5f; // 90 deg over 1.5s
                robot.setTargetVelocity(0.0f, 0.0f, targetOmega);
            }
            break;
        }
        case 2:
            robot.setTargetVelocity(desiredSpeed, 0.0f, 0.0f);
            dx = odometry.getX() - mainDemoStartX;
            dy = odometry.getY() - mainDemoStartY;
            if (sqrt(dx*dx + dy*dy) >= DEMO_FORWARD_DISTANCE)
            {
                state = 3;
                mainDemoStartX = odometry.getX();
                mainDemoStartY = odometry.getY();
                Serial.println("Demo: CRAB_LEFT");
            }
            break;
        case 3:
            robot.setTargetVelocity(0.0f, desiredSpeed, 0.0f);
            dx = odometry.getX() - mainDemoStartX;
            dy = odometry.getY() - mainDemoStartY;
            if (sqrt(dx*dx + dy*dy) >= DEMO_LATERAL_DISTANCE)
            {
                state = 4;
                mainDemoStartX = odometry.getX();
                mainDemoStartY = odometry.getY();
                Serial.println("Demo: MOVE_RIGHT");
            }
            break;
        case 4:
            robot.setTargetVelocity(0.0f, -desiredSpeed, 0.0f);
            dx = odometry.getX() - mainDemoStartX;
            dy = odometry.getY() - mainDemoStartY;
            if (sqrt(dx*dx + dy*dy) >= DEMO_LATERAL_DISTANCE)
            {
                state = 5;
                robot.setTargetVelocity(0.0f, 0.0f, 0.0f);
                mainDemoEnabled = false;
                Serial.println("Demo: DONE");
            }
            break;
        case 5:
            // done
            robot.setTargetVelocity(0.0f, 0.0f, 0.0f);
            break;
    }
}