#ifndef VELOCITYESTIMATOR_H
#define VELOCITYESTIMATOR_H

#include <Arduino.h>
#include "SensorInterface.h"

class VelocityEstimator {
private:
    // Kalman filter state
    float kf_vx;              // Filtered forward velocity (m/s)
    float kf_P;               // State covariance
    
    // Noise parameters (tunable)
    float kf_Q;               // Process noise covariance
    float kf_R_encoder;       // Encoder measurement noise
    float kf_R_imu;           // IMU measurement noise
    
    // IMU velocity integration
    float imu_vx;             // Integrated velocity from IMU
    unsigned long lastUpdate;
    
    // Slip detection
    bool slipDetected;
    float slipThreshold;      // Velocity difference threshold for slip
    float slipAmount;         // Magnitude of slip
    
    // IMU reference
    IMUInterface* imu;
    
    // Helper methods
    void kalmanPredict(float ax, float dt);
    void kalmanUpdate(float encoderVx);
    void updateIMUVelocity(float ax, float dt);
    void detectSlip(float encoderVx);
    
public:
    VelocityEstimator();
    
    // Configuration
    void setIMU(IMUInterface* imuPtr);
    void setNoiseParams(float Q, float R_enc, float R_imu);
    void setSlipThreshold(float threshold);
    
    // Main update - fuses encoder + IMU measurements
    void update(float encoderVx, unsigned long currentTime);
    
    // Getters
    float getVelocity() const { return kf_vx; }
    float getIMUVelocity() const { return imu_vx; }
    bool isSlipping() const { return slipDetected; }
    float getSlipAmount() const { return slipAmount; }
    float getCovariance() const { return kf_P; }
    
    // Reset filter state
    void reset();
};

#endif // VELOCITYESTIMATOR_H
