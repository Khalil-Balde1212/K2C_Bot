# RobotController Library

The RobotController library provides high-level motor control for the K2C_Bot with integrated PID control for both drive and steering motors.

## Features

- **Position PID Control**: Precise steering angle control for pivot motors
- **Speed PID Control**: Accurate RPM control for drive motors
- **Inverse Kinematics Integration**: Convert velocity commands (vx, vy, omega) to wheel speeds and steering angles
- **Modular Design**: Clean separation of motor control logic from main application

## Architecture

### PID Controllers
- **Drive Motors**: Speed control (RPM) with PID
- **Pivot Motors**: Position control (angle in radians) with PID

### Key Classes

#### PIDController
Generic PID controller with:
- Configurable gains (Kp, Ki, Kd)
- Output limits
- Anti-windup protection
- Reset functionality

#### RobotController
Main controller class that:
- Manages four motors (left/right drive, left/right pivot)
- Provides high-level velocity control via inverse kinematics
- Handles PID control loops for all motors
- Offers direct motor control for testing

## Usage

### Basic Setup
```cpp
#include <RobotController.h>

// Create motor objects
Motor leftMotor(12, 13, 2, 7);
Motor leftPivot(15, 14, 11, 8);
Motor rightMotor(5, 4, 3, 4);
Motor rightPivot(7, 6, 10, 9);

// Create robot controller
RobotController robot(&leftMotor, &rightMotor, &leftPivot, &rightPivot);

// Initialize
robot.begin();
```

### High-Level Control
```cpp
// Set velocity (m/s, m/s, rad/s)
robot.setVelocity(0.5, 0.0, 0.0);  // Forward at 0.5 m/s

// Update controller (call in loop)
robot.update(millis());
```

### Direct Motor Control
```cpp
// Set wheel speeds directly (RPM)
robot.setWheelSpeeds(50.0, 50.0);  // Both wheels at 50 RPM

// Set steering angles (radians)
robot.setSteeringAngles(0.0, 0.0);  // Straight ahead
```

### PID Tuning
```cpp
// Tune drive motor PID (speed control)
robot.setDrivePIDGains(2.0, 0.1, 0.05);

// Tune pivot motor PID (position control)
robot.setPivotPIDGains(5.0, 0.0, 0.1);
```

## PID Tuning Guidelines

### Drive Motors (Speed Control)
- **Kp**: Start with 1.0-5.0, increase for faster response
- **Ki**: Start with 0.01-0.1, increase for steady-state accuracy
- **Kd**: Start with 0.01-0.1, increase for oscillation damping

### Pivot Motors (Position Control)
- **Kp**: Start with 2.0-10.0, higher values for stiffer control
- **Ki**: Usually 0 for position control (can cause instability)
- **Kd**: Start with 0.05-0.2, increase for damping oscillations

## Serial Commands

The main application supports these commands:
- `v<speed>`: Set forward velocity (m/s)
- `h<heading>`: Set target heading (degrees)
- `l<rpm>`: Set left wheel speed (RPM)
- `r<rpm>`: Set right wheel speed (RPM)
- `p<angle>`: Set left pivot angle (degrees)
- `q<angle>`: Set right pivot angle (degrees)
- `stop`: Emergency stop
- `reset`: Reset odometry and stop motors

## Integration Notes

- Call `robot.update(currentTime)` regularly in the main loop
- Motor encoder interrupts must be set up separately
- Inverse kinematics integration requires the IK library
- Debug output shows current vs target values for all motors