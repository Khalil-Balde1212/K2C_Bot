# Sensor Interface (IMU & TOF) Library

The SensorInterface library provides a unified IMU and ToF (VL53L0X) sensor API used in the K2C_Bot project.

Components
- `IMUInterface`: Madgwick-based orientation filter + helper methods for gyro/accel/magnetometer.
- `TOF::TOFSensors`: Simple wrapper for multiple VL53L0X sensors with median filtering and offsets.

IMUInterface
- Usage
  - `IMUInterface imu(sampleRate)`: Create an instance with a desired sample rate.
  - `imu.begin()`: Initialize IMU (returns true if present).
  - `imu.update()`: Sample sensors and update filter.
  - `getRoll(), getPitch(), getYaw()`: Get Euler angles (degrees).

- Calibration
  - `calibrateGyro(samples)`: Calibrate gyro with a quiet robot.
  - `calibrateMagnetometer(durationSeconds)`: Magnetometer calibration (rotate robot while collecting samples).
  - `calibrateOrientation()`: Set the current orientation as zero.
  - `setMagCalibration(mx_off, my_off, mz_off, mx_sc, my_sc, mz_sc)`: Use magnetometer calibration values collected offline.

- Advanced
  - `enableBiasEstimation(true)`: Enable online drift bias estimation.
  - `getGyroZ()`, `getAccelX()`, `getAccelY()` for raw axis data.

TOFSensors
- Methods
  - `initialize(pinArray, addrArray, sda, scl)`: Configure XSHUT pins, I2C addresses and bus pins for multiple sensors.
  - `begin()`: Initialize sensors, returns true on success.
  - `readSensor(index)`: Get raw sensor reading.
  - `getFilteredDistance(index)`: Get median-filtered distance in cm.
  - `setSensorOffset(index, mm)`: Set per-sensor offset (useful for mounting misalignments).

Example
```cpp
IMUInterface imu(104.0);
if (imu.begin()) {
    imu.calibrateGyro(100);
    imu.calibrateOrientation();
}

TOF::TOFSensors tof;
int pins[] = {5, 13};
uint8_t addrs[] = {0x30, 0x31};
if (tof.initialize(pins, addrs, A4, A5)) {
    tof.begin();
}

// In main loop
imu.update();
float yaw = imu.getYaw();
float dist0 = tof.getFilteredDistance(0);
```

Notes
- IMU should be calibrated before using orientation results for precise control.
- TOF initialization will try to set unique I2C addresses for multiple sensors.
- Use `getAccelY()` for the forward acceleration when needed for velocity estimation.
