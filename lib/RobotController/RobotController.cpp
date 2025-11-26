#include "RobotController.h"

// PID Controller Implementation
PIDController::PIDController(float p, float i, float d, float min, float max)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f),
      outputMin(min), outputMax(max), lastTime(0) {}

void PIDController::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::setLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
    lastTime = 0;
}

float PIDController::compute(float setpoint, float measurement, unsigned long currentTime) {
    if (lastTime == 0) {
        lastTime = currentTime;
        return 0.0f;
    }

    float dt = (currentTime - lastTime) / 1000.0f;  // Convert to seconds
    if (dt <= 0.0f) return 0.0f;

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
RobotController::RobotController(Motor* lm, Motor* rm, Motor* lp, Motor* rp)
    : leftMotor(lm), rightMotor(rm), leftPivot(lp), rightPivot(rp),
      targetLeftRPM(0.0f), targetRightRPM(0.0f),
      targetLeftAngle(0.0f), targetRightAngle(0.0f),
      initialized(false) {

    // Initialize PID controllers with reasonable defaults
    // Drive motors: Speed PID (RPM control)
    leftDrivePID.setGains(2.0f, 0.1f, 0.05f);
    rightDrivePID.setGains(2.0f, 0.1f, 0.05f);

    // Pivot motors: Position PID (angle control)
    leftPivotPID.setGains(5.0f, 0.0f, 0.1f);
    rightPivotPID.setGains(5.0f, 0.0f, 0.1f);
}

bool RobotController::begin() {
    if (!leftMotor || !rightMotor || !leftPivot || !rightPivot) {
        Serial.println("ERROR: RobotController - Invalid motor pointers");
        return false;
    }

    // Configure motors (basic setup, detailed config done in main)
    leftMotor->setCPR(COUNTS_PER_REV_DRIVE);
    rightMotor->setCPR(COUNTS_PER_REV_DRIVE);
    leftPivot->setCPR(COUNTS_PER_REV_PIVOT);
    rightPivot->setCPR(COUNTS_PER_REV_PIVOT);

    // Stop all motors initially
    stop();

    initialized = true;
    Serial.println("RobotController initialized successfully");
    return true;
}

void RobotController::setVelocity(float vx, float vy, float omega) {
    if (!initialized) return;

    // Use inverse kinematics to convert velocity commands to wheel speeds and steering angles
    IK ik(vx, vy, omega);

    // Set steering angles (convert to radians, assuming IK outputs are in radians)
    setSteeringAngles(ik.theta_left, ik.theta_right);

    // Set wheel speeds (convert to RPM)
    // Average front and rear wheels for each side
    float leftRPM = (ik.wheel_speeds[0] + ik.wheel_speeds[2]) / 2.0f;
    float rightRPM = (ik.wheel_speeds[1] + ik.wheel_speeds[3]) / 2.0f;

    setWheelSpeeds(leftRPM, -rightRPM);  // Right side needs negation
}

void RobotController::setWheelSpeeds(float leftRPM, float rightRPM) {
    targetLeftRPM = leftRPM;
    targetRightRPM = rightRPM;
}

void RobotController::setSteeringAngles(float leftAngle, float rightAngle) {
    targetLeftAngle = leftAngle;
    targetRightAngle = rightAngle;
}

void RobotController::setDrivePIDGains(float kp, float ki, float kd) {
    leftDrivePID.setGains(kp, ki, kd);
    rightDrivePID.setGains(kp, ki, kd);
}

void RobotController::setPivotPIDGains(float kp, float ki, float kd) {
    leftPivotPID.setGains(kp, ki, kd);
    rightPivotPID.setGains(kp, ki, kd);
}

void RobotController::update(unsigned long currentTime) {
    if (!initialized) return;

    // Update drive motors (speed control)
    float currentLeftRPM = *leftMotor->currentRPM();
    float currentRightRPM = *rightMotor->currentRPM();

    int leftDrivePWM = leftDrivePID.compute(targetLeftRPM, currentLeftRPM, currentTime);
    int rightDrivePWM = rightDrivePID.compute(targetRightRPM, currentRightRPM, currentTime);

    leftMotor->setRawSpeed(leftDrivePWM);
    rightMotor->setRawSpeed(rightDrivePWM);

    // Update pivot motors (position control)
    // Convert encoder counts to angles (radians)
    int leftPivotCounts = *leftPivot->getCounts();
    int rightPivotCounts = *rightPivot->getCounts();

    float currentLeftAngle = (leftPivotCounts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
    float currentRightAngle = (rightPivotCounts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;

    int leftPivotPWM = leftPivotPID.compute(targetLeftAngle, currentLeftAngle, currentTime);
    int rightPivotPWM = rightPivotPID.compute(targetRightAngle, currentRightAngle, currentTime);

    leftPivot->setRawSpeed(leftPivotPWM);
    rightPivot->setRawSpeed(rightPivotPWM);

    // Update motor states
    leftMotor->update(currentTime - 10, currentTime);
    rightMotor->update(currentTime - 10, currentTime);
    leftPivot->update(currentTime - 10, currentTime);
    rightPivot->update(currentTime - 10, currentTime);
}

void RobotController::stop() {
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

void RobotController::emergencyStop() {
    // Immediate stop without PID reset
    leftMotor->setRawSpeed(0);
    rightMotor->setRawSpeed(0);
    leftPivot->setRawSpeed(0);
    rightPivot->setRawSpeed(0);
}

float RobotController::getLeftRPM() const {
    return initialized ? *leftMotor->currentRPM() : 0.0f;
}

float RobotController::getRightRPM() const {
    return initialized ? *rightMotor->currentRPM() : 0.0f;
}

float RobotController::getLeftAngle() const {
    if (!initialized) return 0.0f;
    int counts = *leftPivot->getCounts();
    return (counts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
}

float RobotController::getRightAngle() const {
    if (!initialized) return 0.0f;
    int counts = *rightPivot->getCounts();
    return (counts / COUNTS_PER_REV_PIVOT) * 2.0f * PI;
}

void RobotController::printStatus() {
    if (!initialized) {
        Serial.println("RobotController: Not initialized");
        return;
    }

    Serial.print("Drive - Left: ");
    Serial.print(getLeftRPM(), 1);
    Serial.print(" RPM (target: ");
    Serial.print(targetLeftRPM, 1);
    Serial.print(") | Right: ");
    Serial.print(getRightRPM(), 1);
    Serial.print(" RPM (target: ");
    Serial.print(targetRightRPM, 1);
    Serial.println(")");

    Serial.print("Pivot - Left: ");
    Serial.print(getLeftAngle() * 180.0f / PI, 1);
    Serial.print("° (target: ");
    Serial.print(targetLeftAngle * 180.0f / PI, 1);
    Serial.print(") | Right: ");
    Serial.print(getRightAngle() * 180.0f / PI, 1);
    Serial.print("° (target: ");
    Serial.print(targetRightAngle * 180.0f / PI, 1);
    Serial.println(")");
}

int RobotController::rpmToPWM(float rpm) {
    float normalizedRPM = rpm / MAX_RPM;
    int pwm = (int)(normalizedRPM * 4096.0f);
    return constrain(pwm, -4096, 4096);
}

int RobotController::angleToPWM(float angle_rad) {
    // This is a simplified conversion - you may need to tune this
    // based on your specific steering mechanism
    float angle_deg = angle_rad * 57.2958f;
    int pwm = (int)(angle_deg * 45.5f);  // Same as original conversion
    return constrain(pwm, -4096, 4096);
}