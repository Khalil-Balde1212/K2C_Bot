#include "VelocityEstimator.h"

VelocityEstimator::VelocityEstimator()
    : kf_position(0.0f),
      kf_velocity(0.0f),
      kf_R(0.05f),           // Encoder velocity measurement noise
      imu_vx(0.0f),
      imu_position(0.0f),
      lastUpdate(0),
      slipDetected(false),
      slipThreshold(0.15f),  // 0.15 m/s difference indicates slip
      slipAmount(0.0f),
      imu(nullptr)
{
    // Initialize covariance matrix P (state uncertainty)
    kf_P[0][0] = 1.0f;  // Position variance
    kf_P[0][1] = 0.0f;
    kf_P[1][0] = 0.0f;
    kf_P[1][1] = 1.0f;  // Velocity variance
    
    // Initialize process noise Q
    kf_Q[0][0] = 0.001f;  // Position process noise
    kf_Q[0][1] = 0.0f;
    kf_Q[1][0] = 0.0f;
    kf_Q[1][1] = 0.01f;   // Velocity process noise
}

void VelocityEstimator::setIMU(IMUInterface* imuPtr)
{
    imu = imuPtr;
}

void VelocityEstimator::setNoiseParams(float Q_pos, float Q_vel, float R_meas)
{
    kf_Q[0][0] = Q_pos;
    kf_Q[1][1] = Q_vel;
    kf_R = R_meas;
}

void VelocityEstimator::setSlipThreshold(float threshold)
{
    slipThreshold = threshold;
}

void VelocityEstimator::update(float encoderVelocity, unsigned long currentTime)
{
    // Initialize on first call
    if (lastUpdate == 0)
    {
        lastUpdate = currentTime;
        kf_velocity = encoderVelocity;
        imu_vx = 0.0f;
        return;
    }
    
    // Calculate time step
    float dt = (currentTime - lastUpdate) / 1000.0f; // Convert to seconds
    if (dt <= 0.0f || dt > 1.0f) // Sanity check
        return;
    
    // Get IMU acceleration if available
    float ax = 0.0f;
    if (imu != nullptr)
    {
        // IMU orientation: +Y points backward, so forward acceleration is -ay
        ax = -imu->getAccelY();
    }
    
    // 1. Kalman Prediction Step (using IMU acceleration as control input)
    kalmanPredict(ax, dt);
    
    // 2. Update IMU integrated velocity (for comparison/debugging)
    updateIMUVelocity(ax, dt);
    
    // 3. Kalman Update Step (using encoder velocity measurement)
    kalmanUpdate(encoderVelocity, dt);
    
    // 4. Detect slip
    detectSlip(encoderVelocity);
    
    lastUpdate = currentTime;
}

void VelocityEstimator::kalmanPredict(float ax, float dt)
{
    // State prediction: X_k = F * X_k-1 + B * u
    // F = [[1, dt], [0, 1]]  (state transition)
    // B = [[0.5*dt²], [dt]]  (control input matrix)
    // u = ax (IMU acceleration)
    
    float new_position = kf_position + kf_velocity * dt + 0.5f * ax * dt * dt;
    float new_velocity = kf_velocity + ax * dt;
    
    // Covariance prediction: P_k = F * P_k-1 * F^T + Q
    float F[2][2] = {{1.0f, dt}, {0.0f, 1.0f}};
    
    // P_temp = F * P
    float P_temp[2][2];
    P_temp[0][0] = F[0][0] * kf_P[0][0] + F[0][1] * kf_P[1][0];
    P_temp[0][1] = F[0][0] * kf_P[0][1] + F[0][1] * kf_P[1][1];
    P_temp[1][0] = F[1][0] * kf_P[0][0] + F[1][1] * kf_P[1][0];
    P_temp[1][1] = F[1][0] * kf_P[0][1] + F[1][1] * kf_P[1][1];
    
    // P_new = P_temp * F^T + Q
    kf_P[0][0] = P_temp[0][0] * F[0][0] + P_temp[0][1] * F[0][1] + kf_Q[0][0];
    kf_P[0][1] = P_temp[0][0] * F[1][0] + P_temp[0][1] * F[1][1];
    kf_P[1][0] = P_temp[1][0] * F[0][0] + P_temp[1][1] * F[0][1];
    kf_P[1][1] = P_temp[1][0] * F[1][0] + P_temp[1][1] * F[1][1] + kf_Q[1][1];
    
    // Update state
    kf_position = new_position;
    kf_velocity = new_velocity;
}

void VelocityEstimator::kalmanUpdate(float encoderVelocity, float dt)
{
    // Measurement model: H = [[0, 1]] (we measure velocity directly)
    // Innovation: y = z - H * X
    float innovation = encoderVelocity - kf_velocity;
    
    // Innovation covariance: S = H * P * H^T + R
    // Since H = [[0, 1]], this simplifies to S = P[1][1] + R
    float S = kf_P[1][1] + kf_R;
    
    // Kalman gain: K = P * H^T / S
    float K[2];
    K[0] = kf_P[0][1] / S;  // P[0][1] / S
    K[1] = kf_P[1][1] / S;  // P[1][1] / S
    
    // State update: X = X + K * innovation
    kf_position = kf_position + K[0] * innovation;
    kf_velocity = kf_velocity + K[1] * innovation;
    
    // Covariance update: P = (I - K * H) * P
    // Since H = [[0, 1]], this simplifies:
    float P_new[2][2];
    P_new[0][0] = kf_P[0][0] - K[0] * kf_P[1][0];
    P_new[0][1] = kf_P[0][1] - K[0] * kf_P[1][1];
    P_new[1][0] = kf_P[1][0] - K[1] * kf_P[1][0];
    P_new[1][1] = kf_P[1][1] - K[1] * kf_P[1][1];
    
    // Copy back
    kf_P[0][0] = P_new[0][0];
    kf_P[0][1] = P_new[0][1];
    kf_P[1][0] = P_new[1][0];
    kf_P[1][1] = P_new[1][1];
    
    // Ensure covariance doesn't collapse to zero
    if (kf_P[0][0] < 0.0001f) kf_P[0][0] = 0.0001f;
    if (kf_P[1][1] < 0.0001f) kf_P[1][1] = 0.0001f;
}

void VelocityEstimator::updateIMUVelocity(float ax, float dt)
{
    // Simple integration with exponential decay to prevent drift
    const float decay = 0.98f;
    imu_vx = decay * imu_vx + ax * dt;
    imu_position = imu_position + imu_vx * dt;
    
    // Reset if velocity is very small
    if (fabs(imu_vx) < 0.01f && fabs(kf_velocity) < 0.01f)
    {
        imu_vx = 0.0f;
    }
}

void VelocityEstimator::detectSlip(float encoderVelocity)
{
    // Compare encoder velocity vs. Kalman filtered velocity
    // If encoder >> Kalman, wheels are slipping
    slipAmount = encoderVelocity - kf_velocity;
    
    // Detect slip when difference exceeds threshold
    slipDetected = (fabs(slipAmount) > slipThreshold);
}

void VelocityEstimator::reset()
{
    kf_position = 0.0f;
    kf_velocity = 0.0f;
    
    // Reset covariance
    kf_P[0][0] = 1.0f;
    kf_P[0][1] = 0.0f;
    kf_P[1][0] = 0.0f;
    kf_P[1][1] = 1.0f;
    
    imu_vx = 0.0f;
    imu_position = 0.0f;
    slipDetected = false;
    slipAmount = 0.0f;
    lastUpdate = 0;
}
