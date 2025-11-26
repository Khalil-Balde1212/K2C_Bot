#ifndef VELOCITYESTIMATOR_H
#define VELOCITYESTIMATOR_H

#include <Arduino.h>
#include "SensorInterface.h"

class VelocityEstimator {
private:
    // Kalman filter state: [position, velocity]
    float kf_position;        // Estimated position (m)
    float kf_velocity;        // Estimated velocity (m/s)
    float kf_P[2][2];         // 2x2 state covariance matrix
    
    // Noise parameters (tunable)
    float kf_Q[2][2];         // Process noise covariance
    float kf_R;               // Measurement noise (encoder velocity)
    
    // IMU velocity integration (for comparison/debugging)
    float imu_vx;             // Integrated velocity from IMU
    float imu_position;       // Integrated position from IMU
    unsigned long lastUpdate;
    
    // Slip detection
    bool slipDetected;
    float slipThreshold;      // Velocity difference threshold for slip
    float slipAmount;         // Magnitude of slip
    
    // IMU reference
    IMUInterface* imu;
    
    // Helper methods
    void kalmanPredict(float ax, float dt);
    void kalmanUpdate(float encoderVelocity, float dt);
    void updateIMUVelocity(float ax, float dt);
    void detectSlip(float encoderVelocity);
    
public:
    VelocityEstimator();
    
    // Configuration
    void setIMU(IMUInterface* imuPtr);
    void setNoiseParams(float Q_pos, float Q_vel, float R_meas);
    void setSlipThreshold(float threshold);
    
    // Main update - fuses encoder + IMU measurements
    void update(float encoderVelocity, unsigned long currentTime);
    
    // Getters
    float getVelocity() const { return kf_velocity; }
    float getPosition() const { return kf_position; }
    float getIMUVelocity() const { return imu_vx; }
    bool isSlipping() const { return slipDetected; }
    float getSlipAmount() const { return slipAmount; }
    float getCovariance() const { return kf_P[1][1]; }  // Velocity variance
    
    // Reset filter state
    void reset();
};

#endif // VELOCITYESTIMATOR_H
