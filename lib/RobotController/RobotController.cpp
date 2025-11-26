#include "RobotController.h"

// PID Controller Implementation
PIDController::PIDController(float p, float i, float d, float min, float max)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f),
      outputMin(min), outputMax(max), lastTime(0) {}

void PIDController::setGains(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::setLimits(float min, float max)
{
    outputMin = min;
    outputMax = max;
}

void PIDController::reset()
{
    integral = 0.0f;
    prevError = 0.0f;
    lastTime = 0;
}

float PIDController::compute(float setpoint, float measurement, unsigned long currentTime)
{
    if (lastTime == 0)
    {
        lastTime = currentTime;
        return 0.0f;
    }

    float dt = (currentTime - lastTime) / 1000.0f; // Convert to seconds
    if (dt <= 0.0f)
        return 0.0f;

    float error = setpoint - measurement;

    // Proportional term
    float pTerm = kp * error;

    // Integral term
    integral += error * dt;
    // Anti-windup
    integral = constrain(integral, outputMin / ki, outputMax / ki);
    float iTerm = ki * integral;

    // Derivative term
    float derivative = (error - prevError) / dt;
    float dTerm = kd * derivative;

    // Calculate output
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, outputMin, outputMax);

    // Update state
    prevError = error;
    lastTime = currentTime;

    return output;
}

// Robot Controller Implementation
RobotController::RobotController(Motor *lm, Motor *rm, Motor *lp, Motor *rp)
    : leftMotor(lm), rightMotor(rm), leftPivot(lp), rightPivot(rp),
      targetLeftRPM(0.0f), targetRightRPM(0.0f),
      targetLeftAngle(0.0f), targetRightAngle(0.0f),
      leftPivotOffset(0.0f), rightPivotOffset(0.0f),
      initialized(false)
{

    // Initialize PID controllers with reasonable defaults
    // Drive motors: Speed PID (RPM control)
    leftDrivePID.setGains(10.0f, 0.0f, 0.0f);
    rightDrivePID.setGains(10.0f, 0.0f, 0.0f);

    // Pivot motors: Position PID (angle control) - tuned for position control
    leftPivotPID.setGains(15000.0f, 10.0f, 50.0f);
    rightPivotPID.setGains(15000.0f, 10.0f, 50.0f);
}

bool RobotController::begin()
{
    if (!leftMotor || !rightMotor || !leftPivot || !rightPivot)
    {
        Serial.println("ERROR: RobotController - Invalid motor pointers");
        return false;
    }

    // Configure motors (basic setup, detailed config done in main)
    this->leftMotor->setCPR(COUNTS_PER_REV_DRIVE).invertMotor(true);
    this->rightMotor->setCPR(COUNTS_PER_REV_DRIVE).invertEncoder(true);
    this->rightMotor->invertMotor(true);

    this->leftPivot->setCPR(COUNTS_PER_REV_PIVOT);
    this->rightPivot->setCPR(COUNTS_PER_REV_PIVOT);  
    this->leftPivot->invertEncoder(true);
    this->rightPivot->invertEncoder(true);
    // Stop all motors initially
    // stop();

    initialized = true;
    Serial.println("RobotController initialized successfully");
    return true;
}

void RobotController::setWheelSpeeds(float leftRPM, float rightRPM)
{
    targetLeftRPM = leftRPM;
    targetRightRPM = rightRPM;
}

void RobotController::setSteeringAngles(float leftAngle, float rightAngle)
{
    targetLeftAngle = leftAngle;
    targetRightAngle = rightAngle;
}

void RobotController::setDrivePIDGains(float kp, float ki, float kd)
{
    leftDrivePID.setGains(kp, ki, kd);
    rightDrivePID.setGains(kp, ki, kd);
}

void RobotController::setPivotPIDGains(float kp, float ki, float kd)
{
    leftPivotPID.setGains(kp, ki, kd);
    rightPivotPID.setGains(kp, ki, kd);
}

void RobotController::update(unsigned long currentTime)
{
    if (!initialized)
        return;

    // Update drive motors (speed control)
    float currentLeftRPM = *leftMotor->currentRPM();
    float currentRightRPM = *rightMotor->currentRPM();

    int leftDrivePWM = leftDrivePID.compute(targetLeftRPM, currentLeftRPM, currentTime);
    int rightDrivePWM = rightDrivePID.compute(targetRightRPM, currentRightRPM, currentTime);

    leftMotor->setRawSpeed(leftDrivePWM);
    rightMotor->setRawSpeed(rightDrivePWM);

    // Update pivot motors (position control)
    // Convert encoder counts to angles (radians) with calibration offset
    float currentLeftAngle = getLeftAngle();
    float currentRightAngle = getRightAngle();

    int leftPivotPWM = leftPivotPID.compute(targetLeftAngle, currentLeftAngle, currentTime);
    int rightPivotPWM = rightPivotPID.compute(targetRightAngle, currentRightAngle, currentTime);

    leftPivot->setRawSpeed(leftPivotPWM);
    rightPivot->setRawSpeed(rightPivotPWM);

    // Update motor states
    leftMotor->update(currentTime);
    rightMotor->update(currentTime);
    leftPivot->update(currentTime);
    rightPivot->update(currentTime);
}

void RobotController::stop()
{
    targetLeftRPM = 0.0f;
    targetRightRPM = 0.0f;
    targetLeftAngle = 0.0f;
    targetRightAngle = 0.0f;

    leftMotor->setRawSpeed(0);
    rightMotor->setRawSpeed(0);
    leftPivot->setRawSpeed(0);
    rightPivot->setRawSpeed(0);

    // Reset PID controllers
    leftDrivePID.reset();
    rightDrivePID.reset();
    leftPivotPID.reset();
    rightPivotPID.reset();
}

void RobotController::emergencyStop()
{
    // Immediate stop without PID reset
    leftMotor->setRawSpeed(0);
    rightMotor->setRawSpeed(0);
    leftPivot->setRawSpeed(0);
    rightPivot->setRawSpeed(0);
}

float RobotController::getLeftRPM() const
{
    return initialized ? *leftMotor->currentRPM() : 0.0f;
}

float RobotController::getRightRPM() const
{
    return initialized ? *rightMotor->currentRPM() : 0.0f;
}

float RobotController::getLeftAngle() const
{
    if (!initialized)
        return 0.0f;
    int counts = *leftPivot->getCounts();
    float rawAngle = (counts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
    return rawAngle - leftPivotOffset;
}

float RobotController::getRightAngle() const
{
    if (!initialized)
        return 0.0f;
    int counts = *rightPivot->getCounts();
    float rawAngle = (counts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
    return rawAngle - rightPivotOffset;
}

void RobotController::calibratePivotZero()
{
    if (!initialized)
        return;

    int leftCounts = *leftPivot->getCounts();
    int rightCounts = *rightPivot->getCounts();

    leftPivotOffset = (leftCounts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
    rightPivotOffset = (rightCounts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;

    Serial.print("Pivot zero calibrated - Left offset: ");
    Serial.print(leftPivotOffset * 180.0f / PI, 1);
    Serial.print("°, Right offset: ");
    Serial.print(rightPivotOffset * 180.0f / PI, 1);
    Serial.println("°");
}

void RobotController::resetPivotAngles()
{
    targetLeftAngle = getLeftAngle();
    targetRightAngle = getRightAngle();
    Serial.println("Pivot target angles reset to current position");
}

void RobotController::printStatus()
{
    if (!initialized)
    {
        Serial.println("RobotController: Not initialized");
        return;
    }

    Serial.print("Drive - Left:\t");
    Serial.print(getLeftRPM(), 1);
    Serial.print(" RPM (target:\t");
    Serial.print(targetLeftRPM, 1);
    Serial.print(") | Right:\t");
    Serial.print(getRightRPM(), 1);
    Serial.print(" RPM (target:\t");
    Serial.print(targetRightRPM, 1);
    Serial.println(")");

    Serial.print("Pivot - Left:\t");
    Serial.print(getLeftAngle() * 180.0f / PI, 1);
    Serial.print("° (target:\t");
    Serial.print(targetLeftAngle * 180.0f / PI, 1);
    Serial.print(") | Right:\t");
    Serial.print(getRightAngle() * 180.0f / PI, 1);
    Serial.print("° (target:\t");
    Serial.print(targetRightAngle * 180.0f / PI, 1);
    Serial.println(")");
}

int RobotController::rpmToPWM(float rpm)
{
    float normalizedRPM = rpm / MAX_RPM;
    int pwm = (int)(normalizedRPM * 4096.0f);
    return constrain(pwm, -4096, 4096);
}

int RobotController::angleToPWM(float angle_rad)
{
    // This is a simplified conversion - you may need to tune this
    // based on your specific steering mechanism
    float angle_deg = angle_rad * 57.2958f;
    int pwm = (int)(angle_deg * 45.5f); // Same as original conversion
    return constrain(pwm, -4096, 4096);
}