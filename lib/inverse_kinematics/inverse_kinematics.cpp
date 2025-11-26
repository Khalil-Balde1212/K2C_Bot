#include "inverse_kinematics.h"

IK::IK(float vx, float vy, float omega) {
    compute(vx, vy, omega);
}

void IK::compute(float vx, float vy, float omega) {
    // Thresholds
    const float OMEGA_THRESHOLD = 0.5f; // rad/s
    const float VELOCITY_THRESHOLD = 0.01f; // m/s
    
    float v = sqrt(vx*vx + vy*vy);
    float abs_omega = fabs(omega);
    
    // Pivot if velocities are low OR angular velocity is high
    if (v < VELOCITY_THRESHOLD || abs_omega > OMEGA_THRESHOLD) {
        current_mode = PIVOT;
        computePivot(omega);
    }
    // Pure translation
    else if (abs_omega < 1e-6) {
        current_mode = CRAB;
        computeCrab(vx, vy);
    }
    // Combined motion
    else {
        current_mode = ACKERMANN;
        computeAckermann(vx, vy, omega);
    }
    
    // Set steering angles for left/right wheels
    theta_left = theta_s;
    theta_right = -theta_s;
}

void IK::computePivot(float omega) {
    // 45° steering for square robot
    theta_s = 0.7854f * (omega > 0 ? 1.0f : -1.0f);
    
    // All wheels at same speed, alternating signs
    float v_linear = fabs(omega) * r_pivot;
    float sign = (omega > 0) ? 1.0f : -1.0f;
    
    wheel_speeds[0] = metersPerSecToRPM(v_linear * sign);
    wheel_speeds[1] = metersPerSecToRPM(v_linear * -sign);
    wheel_speeds[2] = metersPerSecToRPM(v_linear * sign);
    wheel_speeds[3] = metersPerSecToRPM(v_linear * -sign);
}

void IK::computeCrab(float vx, float vy) {
    // Steer in direction of velocity
    theta_s = atan2(vy, vx);
    
    // All wheels same speed
    float v = sqrt(vx*vx + vy*vy);
    float rpm = metersPerSecToRPM(v);
    
    wheel_speeds[0] = rpm;
    wheel_speeds[1] = rpm;
    wheel_speeds[2] = rpm;
    wheel_speeds[3] = rpm;
}

void IK::computeAckermann(float vx, float vy, float omega) {
    float v = sqrt(vx*vx + vy*vy);
    float R = v / omega;
    
    // Approximate steering angle
    theta_s = atan2(omega * r_pivot, fabs(v));
    if (v < 0) theta_s = -theta_s;
    
    // Inner/outer wheel speeds
    float v_inner = fabs(omega * (fabs(R) - w/2.0f));
    float v_outer = fabs(omega * (fabs(R) + w/2.0f));
    
    if (omega > 0) {
        // CCW: left inner, right outer
        wheel_speeds[0] = metersPerSecToRPM(v_inner);
        wheel_speeds[1] = metersPerSecToRPM(v_outer);
        wheel_speeds[2] = metersPerSecToRPM(v_inner);
        wheel_speeds[3] = metersPerSecToRPM(v_outer);
    } else {
        // CW: right inner, left outer
        wheel_speeds[0] = metersPerSecToRPM(v_outer);
        wheel_speeds[1] = metersPerSecToRPM(v_inner);
        wheel_speeds[2] = metersPerSecToRPM(v_outer);
        wheel_speeds[3] = metersPerSecToRPM(v_inner);
    }
    
    // Reverse all if backing up
    if (v < 0) {
        for (int i = 0; i < 4; i++) {
            wheel_speeds[i] = -wheel_speeds[i];
        }
    }
}