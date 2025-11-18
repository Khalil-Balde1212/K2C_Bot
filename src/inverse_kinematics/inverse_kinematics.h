#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <math.h>

class IK {
private:
    // Your robot parameters
    const float w = 0.074f;        // Track width
    const float l = 0.074f;        // Wheelbase length
    const float r_wheel = 0.00635f; // Wheel radius (12.7mm diameter)
    const float r_pivot = 0.0523f;  // sqrt(w^2+l^2)/2 for pivot mode
    
    // Wheel positions [x, y] relative to robot center
    const float wheel_pos[4][2] = {
        {-0.037f, +0.037f},  // FL
        {+0.037f, +0.037f},  // FR
        {-0.037f, -0.037f},  // RL
        {+0.037f, -0.037f}   // RR
    };
    
    // Mode computation methods (PDF Section 6.3.1)
    void computePivot(float omega);
    void computeCrab(float vx, float vy);
    void computeAckermann(float vx, float vy, float omega);
    
    // Helper to convert m/s to RPM
    float metersPerSecToRPM(float v_ms) {
        return (v_ms / r_wheel) * 9.5493f; // 60/(2π)
    }

public:
    // Operating mode
    enum Mode { PIVOT, CRAB, ACKERMANN };
    Mode current_mode;
    
    // Outputs
    float theta_s;         // Steering control angle (radians)
    float theta_left;      // Left wheels = +theta_s
    float theta_right;     // Right wheels = -theta_s
    float wheel_speeds[4]; // RPM: [FL, FR, RL, RR]
    
    // Constructor - automatically computes IK
    IK(float vx, float vy, float omega);
    
    // Optional: recompute without creating new object
    void compute(float vx, float vy, float omega);
};

#endif //INVERSE_KINEMATICS_H