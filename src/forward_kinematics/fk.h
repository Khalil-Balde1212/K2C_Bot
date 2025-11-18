#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <math.h>

class FK {
private:
    // Robot parameters (same as IK)
    const float w = 0.074f;        // Track width
    const float l = 0.074f;        // Wheelbase length
    const float r_wheel = 0.00635f; // Wheel radius
    
    // Pose state in world frame
    float x;      // X position (m)
    float y;      // Y position (m)
    float psi;    // Heading (radians)
    
    // Velocity in robot frame
    float vx;     // Lateral velocity (m/s)
    float vy;     // Forward velocity (m/s)
    float omega;  // Angular velocity (rad/s)
    
    // Helper: convert RPM to m/s
    float rpmToVelocity(float rpm) {
        return (rpm / 9.5493f) * r_wheel; // Inverse of metersPerSecToRPM
    }

public:
    // Constructor - initializes at origin
    FK();
    
    // Reset pose to origin
    void reset();
    
    // Set initial pose
    void setPose(float x0, float y0, float psi0);
    
    // Update from encoder measurements (PDF Section 8)
    void updateFromEncoders(float delta_s[4], float theta_s, float dt);
    
    // Update from known velocities (PDF Section 5.2)
    void updateFromVelocities(float vx_cmd, float vy_cmd, float omega_cmd, float dt);
    
    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getHeading() const { return psi; }
    float getVx() const { return vx; }
    float getVy() const { return vy; }
    float getOmega() const { return omega; }
};

#endif // FORWARD_KINEMATICS_H