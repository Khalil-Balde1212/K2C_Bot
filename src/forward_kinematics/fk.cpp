#include "fk.h"

FK::FK() {
    reset();
}

void FK::reset() {
    x = 0.0f;
    y = 0.0f;
    psi = 0.0f;
    vx = 0.0f;
    vy = 0.0f;
    omega = 0.0f;
}

void FK::setPose(float x0, float y0, float psi0) {
    x = x0;
    y = y0;
    psi = psi0;
}

// Update from encoder measurements (PDF Section 8)
void FK::updateFromEncoders(float delta_s[4], float theta_s, float dt) {
    // Average displacement (Eq 73)
    float delta_s_avg = (delta_s[0] + delta_s[1] + delta_s[2] + delta_s[3]) / 4.0f;
    
    // Robot frame displacement (Eq 74-75)
    float delta_xR = delta_s_avg * cos(theta_s);
    float delta_yR = delta_s_avg * sin(theta_s);
    
    // Rotation from differential drive (Eq 76)
    float delta_psi = ((delta_s[1] + delta_s[3]) - (delta_s[0] + delta_s[2])) / w;
    
    // Compute velocities in robot frame
    if (dt > 0) {
        vx = delta_xR / dt;
        vy = delta_yR / dt;
        omega = delta_psi / dt;
    }
    
    // World frame update (Eq 77-79)
    x += delta_xR * cos(psi) - delta_yR * sin(psi);
    y += delta_xR * sin(psi) + delta_yR * cos(psi);
    psi += delta_psi;
    
    // Normalize heading to [-π, π]
    while (psi > M_PI) psi -= 2.0f * M_PI;
    while (psi < -M_PI) psi += 2.0f * M_PI;
}

// Update from known velocities (PDF Section 5.2)
void FK::updateFromVelocities(float vx_cmd, float vy_cmd, float omega_cmd, float dt) {
    // Store velocities
    vx = vx_cmd;
    vy = vy_cmd;
    omega = omega_cmd;
    
    // Discrete time update (Eq 43-45)
    x += (vx * cos(psi) - vy * sin(psi)) * dt;
    y += (vx * sin(psi) + vy * cos(psi)) * dt;
    psi += omega * dt;
    
    // Normalize heading
    while (psi > M_PI) psi -= 2.0f * M_PI;
    while (psi < -M_PI) psi += 2.0f * M_PI;
}