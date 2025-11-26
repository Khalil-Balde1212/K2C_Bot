// main.h - Shared application globals and function prototypes
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <RobotController.h>
#include "SensorInterface.h"
#include "fk.h"
#include "MotorInterface.h"

// Extern declarations for globals defined in main.cpp
extern Motor leftMotor;
extern Motor leftPivot;
extern Motor rightMotor;
extern Motor rightPivot;
extern RobotController robot;
extern IMUInterface imu;
extern bool imuAvailable;
extern FK odometry;
extern TOF::TOFSensors tofSensors;

// State variables from main.cpp
extern unsigned long lastTime;
extern float targetHeading;
extern float desiredSpeed;
extern bool quietMode;
extern float headingErrorIntegral;
extern float lastHeadingError;
extern int lastLeftCounts;
extern int lastRightCounts;
extern float lastCmdVx;
extern float lastCmdVy;
extern float lastCmdOmega;

// Functions moved out of main or used across files
void serialCommands();
void debugPrints(unsigned long currentTime);
void startMainDemo();
void stopMainDemo();
void restartMainDemo();
void updateMainDemo(unsigned long currentTime);
// Demo globals
extern bool mainDemoEnabled;
extern float mainDemoStartX;
extern float mainDemoStartY;
extern float mainDemoStartYaw;
extern unsigned long lastOdometryUpdate;
// Demo parameters
extern const float DEMO_FORWARD_DISTANCE;
extern const float DEMO_LATERAL_DISTANCE;
extern const float DEMO_TURN_ANGLE;

#endif // MAIN_H
