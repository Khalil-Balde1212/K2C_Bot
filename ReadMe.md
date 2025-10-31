# dev_SI Branch - Sensor Interface Development

## Overview
Development branch for IMU and TOF sensor interface implementation and testing.

## Components

### IMU (Arduino_BMI270_BMM150)
- 9-axis sensor with accelerometer, gyroscope, and magnetometer
- Madgwick AHRS filter for orientation estimation

### TOF (VL53L0X)
- Time-of-flight distance sensors
- Support for up to 4 sensors with address multiplexing
- Median filtering for noise reduction

## Key Findings

### IMU Development

**Magnetometer Issues**
- Magnetometer readings showed minimal variation during rotation
- Likely caused by magnetic interference from electronics/motors
- Solution: Switched to gyro-only mode using `filter.updateIMU()` instead of `filter.update()`

**Gyroscope Integration**
- Arduino MadgwickAHRS library expects gyroscope input in degrees/second, not radians/second
- Converting to radians caused 57x slower integration
- Fixed by removing `DEG_TO_RAD` conversion

**Sample Rate Synchronization**
- Filter must be initialized with actual update rate
- Mismatch between filter rate and loop rate caused incorrect integration
- Example: 100 Hz filter with 20 Hz updates results in 5x slower rotation tracking

**Calibration**
- Implemented gyroscope bias calibration on startup
- Collects 100 samples over ~1 second while stationary
- Orientation zeroing allows starting from 0,0,0 at any position

### TOF Development

**I2C Initialization Bug**
- Original code: `sdaPin || sclPin ? Wire.begin(sdaPin, sclPin) : Wire.begin()`
- Boolean OR evaluated incorrectly with default pin values of 0
- Fixed with explicit non-zero check: `if (sdaPin != 0 || sclPin != 0)`

**Known Issues**
- Multiple TOF sensors not initializing reliably
- Sensor 0 works consistently, sensors 1-3 return timeout errors
- Possible I2C bus conflicts or timing issues during sequential initialization

## Code Structure

### Files
- `SensorInterface.h` - Class declarations and public interface
- `SensorInterface.cpp` - Implementation of IMU and TOF functionality

### Classes
- `IMUInterface` - Manages BMI270 IMU with Madgwick filtering
- `TOF::MedianFilter` - 5-sample median filter for distance measurements
- `TOF::TOFSensors` - Manages multiple VL53L0X sensors with multiplexing

## Usage Example
```cpp
#include <SensorInterface.h>

IMUInterface imu(100.0);  // 100 Hz update rate
TOF::TOFSensors tofSensors;

void setup() {
    imu.begin();
    imu.calibrateGyro(100);
    imu.calibrateOrientation();
    
    const int xshutPins[] = {2, 3, 4, 5};
    const uint8_t addresses[] = {0x30, 0x31, 0x32, 0x33};
    tofSensors.initialize(xshutPins, addresses, 0, 0);
    tofSensors.begin();
}

void loop() {
    imu.update();
    float yaw = imu.getYaw();
    float distance = tofSensors.getFilteredDistance(0);
}
```

## Next Steps
- Debug TOF sensor initialization for multi-sensor arrays
- Test long-term gyroscope drift characteristics
- Consider implementing complementary filter as alternative to Madgwick