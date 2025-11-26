#include "VelocityEstimator.h"

VelocityEstimator::VelocityEstimator()
    : kf_vx(0.0f),
      kf_P(1.0f),
      kf_Q(0.01f),           // Process noise - tune based on robot dynamics
      kf_R_encoder(0.05f),   // Encoder noise - relatively low
      kf_R_imu(0.2f),        // IMU noise - higher due to integration drift
      imu_vx(0.0f),
      lastUpdate(0),
      slipDetected(false),
      slipThreshold(0.15f),  // 0.15 m/s difference indicates slip
      slipAmount(0.0f),
      imu(nullptr)
{
}

void VelocityEstimator::setIMU(IMUInterface* imuPtr)
{
    imu = imuPtr;
}

void VelocityEstimator::setNoiseParams(float Q, float R_enc, float R_imu)
{
    kf_Q = Q;
    kf_R_encoder = R_enc;
    kf_R_imu = R_imu;
}

void VelocityEstimator::setSlipThreshold(float threshold)
{
    slipThreshold = threshold;
}

void VelocityEstimator::update(float encoderVx, unsigned long currentTime)
{
    // Initialize on first call
    if (lastUpdate == 0)
    {
        lastUpdate = currentTime;
        kf_vx = encoderVx;
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
        // IMPORTANT: IMU orientation on Nano 33 BLE 
        // +Y points backward, so forward acceleration is -ay
        ax = -imu->getAccelY();
    }
    
    // 1. Kalman Prediction Step (using IMU acceleration)
    kalmanPredict(ax, dt);
    
    // 2. Update IMU integrated velocity
    updateIMUVelocity(ax, dt);
    
    // 3. Kalman Update Step (fusing encoder measurement)
    kalmanUpdate(encoderVx);
    
    // 4. Detect slip
    detectSlip(encoderVx);
    
    lastUpdate = currentTime;
}

void VelocityEstimator::kalmanPredict(float ax, float dt)
{
    // State prediction: vx(k+1) = vx(k) + ax * dt
    kf_vx = kf_vx + ax * dt;
    
    // Covariance prediction: P(k+1) = P(k) + Q
    kf_P = kf_P + kf_Q;
}

void VelocityEstimator::kalmanUpdate(float encoderVx)
{
    // Innovation (measurement residual)
    float y = encoderVx - kf_vx;
    
    // Innovation covariance
    float S = kf_P + kf_R_encoder;
    
    // Kalman gain
    float K = kf_P / S;
    
    // State update
    kf_vx = kf_vx + K * y;
    
    // Covariance update
    kf_P = (1.0f - K) * kf_P;
    
    // Ensure covariance doesn't go to zero (numerical stability)
    if (kf_P < 0.001f)
        kf_P = 0.001f;
}

void VelocityEstimator::updateIMUVelocity(float ax, float dt)
{
    // Simple integration with exponential decay to prevent drift
    // This provides a second velocity estimate for comparison
    const float decay = 0.98f; // Slight decay to prevent unbounded drift
    imu_vx = decay * imu_vx + ax * dt;
    
    // Reset if velocity is very small (robot stopped)
    if (fabs(imu_vx) < 0.01f && fabs(kf_vx) < 0.01f)
    {
        imu_vx = 0.0f;
    }
}

void VelocityEstimator::detectSlip(float encoderVx)
{
    // Compare encoder velocity vs. Kalman filtered velocity
    // If encoder >> Kalman, wheels are slipping
    slipAmount = encoderVx - kf_vx;
    
    // Detect slip when difference exceeds threshold
    if (fabs(slipAmount) > slipThreshold)
    {
        slipDetected = true;
    }
    else
    {
        slipDetected = false;
    }
}

void VelocityEstimator::reset()
{
    kf_vx = 0.0f;
    kf_P = 1.0f;
    imu_vx = 0.0f;
    slipDetected = false;
    slipAmount = 0.0f;
    lastUpdate = 0;
}
