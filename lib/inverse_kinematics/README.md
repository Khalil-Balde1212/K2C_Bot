# Inverse Kinematics (IK) Library

The IK library converts desired robot-frame velocities (vx, vy, omega) into per-wheel speeds and steering angles for a 4-wheel swerve-like platform. It supports three modes:

Modes
- PIVOT — turn on the spot (low speed or high angular commands)
- CRAB — lateral translation, steering angles are parallel
- ACKERMANN — nominal forward driving with differential inner/outer wheel speeds

Features
- Automatic mode selection based on velocity vector direction
- Returns steering command (theta_s) and per-wheel RPMs
- Simple helper: `metersPerSecToRPM` to convert m/s to RPM

Class: `IK`

Public fields
- `Mode current_mode` — current operating mode (PIVOT, CRAB, ACKERMANN)
- `float theta_s` — steering target for the wheels (radians)
- `float theta_left, theta_right` — per-side steering values (radians)
- `float wheel_speeds[4]` — per-wheel RPMs in order `[FL, FR, RL, RR]`

Constructor & methods
- `IK(float vx, float vy, float omega);` — construct and compute outputs
- `void compute(float vx, float vy, float omega);` — recompute outputs

Notes
- The IK internally chooses mode by looking at the velocity angle (atan2(vy, vx)).
  - ackerman: |theta_vel| < 10°
  - pivot: 10° to 80°
  - crab: |theta_vel| >= 80°
- `vx`, `vy` are robot-frame velocities (m/s). `omega` is angular velocity (rad/s).
- The library is intentionally lightweight; use RobotController to integrate and command motors.

Example
```cpp
// Compute IK for forward motion
IK ik(0.16, 0.0, 0.0);
// Apply steering and wheel RPMs from ik
robot.setSteeringAngles(ik.theta_left, ik.theta_right);
float leftRPM = (ik.wheel_speeds[0] + ik.wheel_speeds[2]) / 2.0f;
float rightRPM = (ik.wheel_speeds[1] + ik.wheel_speeds[3]) / 2.0f;
robot.setWheelSpeeds(leftRPM, rightRPM);
```

Recommended tuning
- Use the RobotController's `enableVelocityControl(true)` and `setTargetVelocity(vx, vy, omega)` to take advantage of higher-level smoothing and PID control.