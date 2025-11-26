# VelocityEstimator Library

The VelocityEstimator provides a small Kalman-filter based estimator to fuse encoder measurements with IMU data for robust robot velocity estimation, plus slip detection.

Purpose
- Fuse encoder velocity with IMU integration for improved accuracy and robustness to slip and noise.
- Expose a single-source `getVelocity()` and `getPosition()` used by RobotController velocity control.

Class `VelocityEstimator` API
- `VelocityEstimator()` — constructor
- `void setIMU(IMUInterface* imuPtr);` — pass the IMU reference to integrate and compare acceleration-based velocities
- `void setNoiseParams(float Q_pos, float Q_vel, float R_meas);` — tune Kalman process/measurement noise
- `void setSlipThreshold(float threshold);` — set slip detection threshold (m/s difference)
- `void update(float encoderVelocity, unsigned long currentTime);` — call with encoder measurement in m/s and the current time (millis())
- `float getVelocity() const;` — returns the Kalman estimated velocity (m/s)
- `float getPosition() const;` — estimated position from the filter
- `float getIMUVelocity() const;` — integrated IMU-derived velocity for diagnostics
- `bool isSlipping() const;` — indicates whether slip is detected
- `float getSlipAmount() const;` — magnitude of slip detected
- `void reset();` — reset Kalman filter state

Notes on use
- Call `setIMU()` with your IMU instance if available; otherwise it will operate in encoder-only mode.
- `update()` takes the encoder velocity in m/s and uses the internal timestamp to compute `dt` between calls.
- Tuning `Q` and `R` is key to balancing responsiveness against noise. Defaults are conservative for small robots.

Example
```cpp
VelocityEstimator estimator;
estimator.setIMU(&imu);
estimator.setNoiseParams(0.01f, 0.1f, 0.01f);

// In loop
estimator.update(encoderVx, millis());
float vx = estimator.getVelocity();
if (estimator.isSlipping()) {
    // handle slip condition (reduce power or abort motion)
}
```
