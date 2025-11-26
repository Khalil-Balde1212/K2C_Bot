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

    // Integral term with anti-windup
    integral += error * dt;
    
    // Anti-windup: Clamp integral to prevent windup
    // Calculate max integral contribution based on output limits
    if (ki != 0.0f)
    {
        float maxIntegral = (outputMax - pTerm) / ki;
        float minIntegral = (outputMin - pTerm) / ki;
        integral = constrain(integral, minIntegral, maxIntegral);
    }
    else
    {
        integral = 0.0f;  // If ki is 0, integral term doesn't matter
    }
    
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
      initialized(false),
      targetVx(0.0f), targetVy(0.0f), targetOmega(0.0f),
      currentVx(0.0f), currentVy(0.0f), currentOmega(0.0f),
      rawEncoderVx(0.0f),
      velocityControlEnabled(false),
      lastVelocityUpdate(0),
      lastLeftDriveCounts(0), lastRightDriveCounts(0),
      imu(nullptr)
{

    // Initialize PID controllers with reasonable defaults
    // Drive motors: Speed PID (RPM control)
    leftDrivePID.setGains(1.0f, 0.0f, 0.00f);
    rightDrivePID.setGains(1.0f, 0.0f, 0.00f);

    // Pivot motors: Position PID (angle control) - tuned for position control
    leftPivotPID.setGains(15000.0f, 10.0f, 50.0f);
    rightPivotPID.setGains(15000.0f, 10.0f, 50.0f);
    
    // Velocity control PIDs: Robot-level velocity control
    // Using very conservative gains to start - user can tune with velgains command
    vxPID.setGains(5.0f, 0.5f, 0.1f);      // Forward/backward velocity (reduced from 50, 5, 2)
    vyPID.setGains(5.0f, 0.5f, 0.1f);      // Lateral velocity (crab mode)
    omegaPID.setGains(2.0f, 0.2f, 0.05f);  // Angular velocity (reduced from 20, 2, 1)
    
    // Set output limits for velocity PIDs (output is velocity correction in m/s or rad/s)
    vxPID.setLimits(-0.5f, 0.5f);       // +/- 0.5 m/s correction
    vyPID.setLimits(-0.5f, 0.5f);       // +/- 0.5 m/s correction
    omegaPID.setLimits(-1.0f, 1.0f);    // +/- 1.0 rad/s correction
}

bool RobotController::begin()
{
    if (!leftMotor || !rightMotor || !leftPivot || !rightPivot)
    {
        Serial.println("ERROR: RobotController - Invalid motor pointers");
        return false;
    }

    // Configure motors (basic setup, detailed config done in main)
    this->leftMotor->setCPR(2200.0f).invertMotor(true);
    this->rightMotor->setCPR(2200.0f).invertEncoder(true);
    this->rightMotor->invertMotor(true);

    this->leftPivot->setCPR(1440.0f);
    this->rightPivot->setCPR(1440.0f);  
    this->leftPivot->invertEncoder(true);
    this->rightPivot->invertEncoder(true);
    // Stop all motors initially
    // stop();

    initialized = true;
    Serial.println("RobotController initialized successfully");
    return true;
}

void RobotController::setVelocity(float vx, float vy, float omega)
{
    if (!initialized)
        return;

    // Use inverse kinematics to convert velocity commands to wheel speeds and steering angles
    IK ik(vx, vy, omega);

    // Set steering angles (convert to radians, assuming IK outputs are in radians)
    setSteeringAngles(ik.theta_left, ik.theta_right);

    // Set wheel speeds (convert to RPM)
    // Average front and rear wheels for each side
    float leftRPM = (ik.wheel_speeds[0] + ik.wheel_speeds[2]) / 2.0f;
    float rightRPM = (ik.wheel_speeds[1] + ik.wheel_speeds[3]) / 2.0f;

    setWheelSpeeds(leftRPM, -rightRPM); // Right side needs negation
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

void RobotController::setVelocityControlGains(float kp_vx, float ki_vx, float kd_vx,
                                               float kp_vy, float ki_vy, float kd_vy,
                                               float kp_omega, float ki_omega, float kd_omega)
{
    vxPID.setGains(kp_vx, ki_vx, kd_vx);
    vyPID.setGains(kp_vy, ki_vy, kd_vy);
    omegaPID.setGains(kp_omega, ki_omega, kd_omega);
}

void RobotController::update(unsigned long currentTime)
{
    if (!initialized)
        return;

    // Always update velocity estimate for monitoring (even if control disabled)
    updateVelocityEstimate(currentTime);
    
    // Run closed-loop velocity control if enabled
    if (velocityControlEnabled)
    {
        updateVelocityControl(currentTime);
    }

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

void RobotController::enableVelocityControl(bool enable)
{
    if (enable && !velocityControlEnabled)
    {
        // Entering velocity control mode - initialize state
        velocityControlEnabled = true;
        lastVelocityUpdate = millis();
        lastLeftDriveCounts = *leftMotor->getCounts();
        lastRightDriveCounts = *rightMotor->getCounts();
        currentVx = 0.0f;
        currentVy = 0.0f;
        currentOmega = 0.0f;
        vxPID.reset();
        vyPID.reset();
        omegaPID.reset();
        Serial.println("Velocity control ENABLED");
    }
    else if (!enable && velocityControlEnabled)
    {
        // Exiting velocity control mode - reset all targets to zero
        velocityControlEnabled = false;
        targetVx = 0.0f;
        targetVy = 0.0f;
        targetOmega = 0.0f;
        
        // Stop the motors
        setWheelSpeeds(0.0f, 0.0f);
        setSteeringAngles(0.0f, 0.0f);
        
        Serial.println("Velocity control DISABLED");
    }
}

void RobotController::setTargetVelocity(float vx, float vy, float omega)
{
    targetVx = vx;
    targetVy = vy;
    targetOmega = omega;
}

void RobotController::setIMUReference(IMUInterface* imuPtr)
{
    imu = imuPtr;  // Store for debug output
    velocityEstimator.setIMU(imuPtr);
}

void RobotController::updateVelocityEstimate(unsigned long currentTime)
{
    if (!initialized)
        return;

    // Initialize on first call
    if (lastVelocityUpdate == 0)
    {
        lastVelocityUpdate = currentTime;
        lastLeftDriveCounts = *leftMotor->getCounts();
        lastRightDriveCounts = *rightMotor->getCounts();
        return;
    }

    float dt = (currentTime - lastVelocityUpdate) / 1000.0f; // Convert to seconds
    
    // Don't update too frequently - minimum 50ms between updates to reduce noise
    if (dt < 0.05f)
        return;
    
    if (dt <= 0.0f)
        return;

    // Get current encoder counts
    int currentLeftCounts = *leftMotor->getCounts();
    int currentRightCounts = *rightMotor->getCounts();

    // Calculate wheel velocities from encoder changes
    int leftDelta = currentLeftCounts - lastLeftDriveCounts;
    int rightDelta = currentRightCounts - lastRightDriveCounts;

    // Convert encoder counts to wheel velocities (m/s)
    float leftWheelVel = (leftDelta / COUNTS_PER_REV_DRIVE) * (2.0f * PI * WHEEL_RADIUS) / dt;
    float rightWheelVel = (rightDelta / COUNTS_PER_REV_DRIVE) * (2.0f * PI * WHEEL_RADIUS) / dt;

    // Differential drive kinematics
    const float WHEELBASE = 0.074f; // meters
    
    // Calculate encoder-based forward velocity
    float encoderVx = (leftWheelVel + rightWheelVel) / 2.0f;
    float rawOmega = (rightWheelVel - leftWheelVel) / WHEELBASE;
    
    // Store raw encoder velocity for debug/slip detection
    rawEncoderVx = encoderVx;
    
    // Update Kalman filter with encoder measurement
    velocityEstimator.update(encoderVx, currentTime);
    
    // TEMPORARY: Use raw encoder velocity directly (Kalman filter has bugs)
    // TODO: Debug Kalman filter - it's not properly fusing encoder + IMU
    currentVx = encoderVx;  // Use raw encoder instead of velocityEstimator.getVelocity()
    
    // Apply smoothing filter to omega
    const float alpha = 0.3f;
    currentOmega = alpha * rawOmega + (1.0f - alpha) * currentOmega;
    
    // Lateral velocity (vy) - simplified for differential drive
    currentVy = 0.0f;

    // Update encoder counts for next iteration
    lastLeftDriveCounts = currentLeftCounts;
    lastRightDriveCounts = currentRightCounts;
    lastVelocityUpdate = currentTime;
}

void RobotController::updateVelocityControl(unsigned long currentTime)
{
    if (!initialized || !velocityControlEnabled)
        return;

    // Update velocity estimate
    updateVelocityEstimate(currentTime);

    // Compute PID corrections
    float vxCorrection = vxPID.compute(targetVx, currentVx, currentTime);
    float omegaCorrection = omegaPID.compute(targetOmega, currentOmega, currentTime);

    // Apply corrections to target velocities (vy not used in differential drive)
    float correctedVx = targetVx + vxCorrection;
    float correctedOmega = targetOmega + omegaCorrection;

    // Convert corrected velocities to wheel speeds using simplified differential drive model
    // For swerve drive, we should NOT recalculate steering angles in the control loop
    // Instead, set steering angles once at the beginning, then only adjust wheel speeds
    
    // Use differential drive approximation: 
    // Left wheel velocity = vx - omega * (wheelbase/2)
    // Right wheel velocity = vx + omega * (wheelbase/2)
    const float WHEELBASE = 0.074f; // meters
    const float wheelCircumference = 2.0f * PI * WHEEL_RADIUS;
    
    float leftWheelSpeed = correctedVx - (correctedOmega * WHEELBASE / 2.0f);  // m/s
    float rightWheelSpeed = correctedVx + (correctedOmega * WHEELBASE / 2.0f); // m/s
    
    // Convert to RPM: (m/s) / (m/rev) * 60 s/min = RPM
    // Negate to match motor/encoder convention (positive RPM = forward)
    float leftRPM = -(leftWheelSpeed / wheelCircumference) * 60.0f;
    float rightRPM = -(rightWheelSpeed / wheelCircumference) * 60.0f;
    
    // Set wheel speeds directly without changing steering angles
    // Note: Right side negation is already handled in setWheelSpeeds for inverse kinematics mode
    // In velocity control mode (differential drive), we want both wheels spinning the same direction for forward
    setWheelSpeeds(leftRPM, rightRPM);
    
    // Set steering to straight ahead (0 degrees) when in velocity control mode
    // This prevents the steering from oscillating
    setSteeringAngles(0.0f, 0.0f);
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
    
    // Show velocity control status if enabled
    if (velocityControlEnabled)
    {
        Serial.print("Vx:\t");
        Serial.print(currentVx, 3);
        Serial.print(" | Enc_raw: ");
        Serial.print(rawEncoderVx, 3);
        Serial.print(" | KF_vx: ");
        Serial.print(velocityEstimator.getVelocity(), 3);
        Serial.print(" | IMU_vx: ");
        Serial.print(velocityEstimator.getIMUVelocity(), 3);
        
        // DEBUG: Show raw IMU acceleration to verify it's being read
        if (imu != nullptr)
        {
            Serial.print(" | IMU_ay: ");
            Serial.print(-imu->getAccelY(), 3);  // Negated for forward
        }
        
        Serial.print(" | Slip: ");
        Serial.print(velocityEstimator.getSlipAmount(), 3);
        
        if (velocityEstimator.isSlipping())
        {
            Serial.print(" ⚠️");
        }
        
        Serial.println();
    }
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