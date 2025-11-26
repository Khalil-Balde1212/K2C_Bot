# Forward Kinematics (FK) Library

The FK library converts wheel displacements and robot-frame velocities into a world-frame odometry estimate (x, y, heading) and provides helper methods for estimating vehicle motion.

Key features
- Simple, small-footprint forward kinematics for the K2C platform
- Supports: update from encoder deltas via `updateFromEncoders` or velocities via `updateFromVelocities`
- Returns pose (x, y, heading) and velocities (vx, vy, omega)

Class: `FK`

Public methods
- `FK()` — constructor
- `void reset()` — reset pose and velocities
- `void setPose(float x0, float y0, float psi0)` — set initial world pose (m and radians)
- `void updateFromEncoders(float delta_s[4], float theta_s, float dt)` — update pose from 4-wheel displacement (m) and steering angle
- `void updateFromVelocities(float vx_cmd, float vy_cmd, float omega_cmd, float dt)` — integrate pose from commanded velocities (robot frame)
- `float getX() const, getY(), getHeading()` — pose getters
- `float getVx(), getVy(), getOmega()` — velocity getters

Notes and units
- Positions are in meters (m); heading is in radians.
- Velocities are in m/s (forward vx, lateral vy) and angular speed in rad/s.
- The library uses a simple discrete-time integration; for best accuracy, call update at a regular rate and supply measured dt.

Example usage
```cpp
FK odometry;

void loop() {
    // If you have velocities from higher-level controller
    float vx = robot.getCurrentVx();
    float vy = robot.getCurrentVy();
    float omega = robot.getCurrentOmega();
    float dt = (millis() - lastTime) / 1000.0f;
    odometry.updateFromVelocities(vx, vy, omega, dt);
}
```

Where to use
- The FK library is useful when you want a simple onboard odometry estimate driven by velocity commands or encoder deltas.