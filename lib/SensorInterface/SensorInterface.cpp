#include <SensorInterface.h>

IMUInterface::IMUInterface(float sampleRate)
    : sensorRate(sampleRate),
      filter(),
      ax(0.0f), ay(0.0f), az(0.0f),
      gx(0.0f), gy(0.0f), gz(0.0f),
      mx(0.0f), my(0.0f), mz(0.0f),
      roll(0.0f), pitch(0.0f), heading(0.0f) {
    filter.begin(sensorRate);
}

bool IMUInterface::begin() {
    if (!IMU.begin()) {
        return false;
    }
    return true;
}
bool IMUInterface::calibrateGyro(int samples) {
    Serial.println("Calibrating gyroscope...");
    Serial.println("Keep device COMPLETELY STILL!");
    delay(2000);  // Give user time to set it down
    
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    
    for (int i = 0; i < samples; i++) {
        if (IMU.gyroscopeAvailable()) {
            float temp_gx, temp_gy, temp_gz;
            IMU.readGyroscope(temp_gx, temp_gy, temp_gz);
            sum_gx += temp_gx;
            sum_gy += temp_gy;
            sum_gz += temp_gz;
        }
        delay(10);
        
        if (i % 20 == 0) {
            Serial.print(".");
        }
    }
    
    gx_bias = sum_gx / samples;
    gy_bias = sum_gy / samples;
    gz_bias = sum_gz / samples;
    
    Serial.println();
    Serial.print("Gyro bias: gx=");
    Serial.print(gx_bias);
    Serial.print(" gy=");
    Serial.print(gy_bias);
    Serial.print(" gz=");
    Serial.println(gz_bias);
    Serial.println("Calibration complete!");
    
    return true;
}

bool IMUInterface::calibrateMagnetometer(int durationSeconds) {
    Serial.println("Magnetometer calibration starting...");
    Serial.println("Slowly rotate the robot 360° in all orientations!");
    Serial.println("Roll, pitch, and yaw the robot to cover all angles.");
    delay(2000);

    float mx_min = 9999.0f, mx_max = -9999.0f;
    float my_min = 9999.0f, my_max = -9999.0f;
    float mz_min = 9999.0f, mz_max = -9999.0f;

    unsigned long startTime = millis();
    unsigned long duration = durationSeconds * 1000UL;
    int samples = 0;

    Serial.print("Calibrating for ");
    Serial.print(durationSeconds);
    Serial.println(" seconds...");

    while (millis() - startTime < duration) {
        if (IMU.magneticFieldAvailable()) {
            float temp_mx, temp_my, temp_mz;
            IMU.readMagneticField(temp_mx, temp_my, temp_mz);

            // Track min/max for each axis
            if (temp_mx < mx_min) mx_min = temp_mx;
            if (temp_mx > mx_max) mx_max = temp_mx;
            if (temp_my < my_min) my_min = temp_my;
            if (temp_my > my_max) my_max = temp_my;
            if (temp_mz < mz_min) mz_min = temp_mz;
            if (temp_mz > mz_max) mz_max = temp_mz;

            samples++;
        }

        // Progress indicator every second
        if ((millis() - startTime) % 1000 < 50) {
            int remaining = durationSeconds - (millis() - startTime) / 1000;
            Serial.print(remaining);
            Serial.print("s remaining | Samples: ");
            Serial.print(samples);
            Serial.print(" | X:[");
            Serial.print(mx_min, 1);
            Serial.print(",");
            Serial.print(mx_max, 1);
            Serial.print("] Y:[");
            Serial.print(my_min, 1);
            Serial.print(",");
            Serial.print(my_max, 1);
            Serial.print("] Z:[");
            Serial.print(mz_min, 1);
            Serial.print(",");
            Serial.print(mz_max, 1);
            Serial.println("]");
            delay(50);
        }

        delay(50);  // ~20Hz magnetometer rate
    }

    // Compute hard iron offsets (center of the ellipsoid)
    mx_offset = (mx_max + mx_min) / 2.0f;
    my_offset = (my_max + my_min) / 2.0f;
    mz_offset = (mz_max + mz_min) / 2.0f;

    // Compute soft iron scale factors (normalize to sphere)
    float mx_range = (mx_max - mx_min) / 2.0f;
    float my_range = (my_max - my_min) / 2.0f;
    float mz_range = (mz_max - mz_min) / 2.0f;
    float avg_range = (mx_range + my_range + mz_range) / 3.0f;

    if (mx_range > 0) mx_scale = avg_range / mx_range;
    if (my_range > 0) my_scale = avg_range / my_range;
    if (mz_range > 0) mz_scale = avg_range / mz_range;

    Serial.println("\n=== Magnetometer Calibration Complete ===");
    Serial.print("Hard iron offsets: X=");
    Serial.print(mx_offset, 2);
    Serial.print(" Y=");
    Serial.print(my_offset, 2);
    Serial.print(" Z=");
    Serial.println(mz_offset, 2);
    Serial.print("Soft iron scales:  X=");
    Serial.print(mx_scale, 3);
    Serial.print(" Y=");
    Serial.print(my_scale, 3);
    Serial.print(" Z=");
    Serial.println(mz_scale, 3);
    Serial.print("Total samples: ");
    Serial.println(samples);

    return samples > 50;  // Need reasonable number of samples
}

void IMUInterface::calibrateOrientation() {
    rollOffset = roll;
    pitchOffset = pitch;
    headingOffset = heading;
    Serial.println("Orientation zeroed at current position.");
}

void IMUInterface::resetCalibration() {
    gx_bias = 0.0f;
    gy_bias = 0.0f;
    gz_bias = 0.0f;
    rollOffset = 0.0f;
    pitchOffset = 0.0f;
    headingOffset = 0.0f;
    mx_offset = 0.0f;
    my_offset = 0.0f;
    mz_offset = 0.0f;
    mx_scale = 1.0f;
    my_scale = 1.0f;
    mz_scale = 1.0f;
    Serial.println("All calibration cleared.");
}

void IMUInterface::setMagCalibration(float mx_off, float my_off, float mz_off,
                                      float mx_sc, float my_sc, float mz_sc) {
    mx_offset = mx_off;
    my_offset = my_off;
    mz_offset = mz_off;
    mx_scale = mx_sc;
    my_scale = my_sc;
    mz_scale = mz_sc;

    Serial.println("Magnetometer calibration set:");
    Serial.print("  Hard iron: X=");
    Serial.print(mx_offset, 2);
    Serial.print(" Y=");
    Serial.print(my_offset, 2);
    Serial.print(" Z=");
    Serial.println(mz_offset, 2);
    Serial.print("  Soft iron: X=");
    Serial.print(mx_scale, 3);
    Serial.print(" Y=");
    Serial.print(my_scale, 3);
    Serial.print(" Z=");
    Serial.println(mz_scale, 3);
}

// Update the update() function to use bias correction:
void IMUInterface::update() {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
    }
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);

        gx -= gx_bias;
        gy -= gy_bias;
        gz -= gz_bias;

        // Use Madgwick for roll/pitch (6-DOF is fine for these)
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        roll = filter.getRoll();
        pitch = filter.getPitch();

        // Apply magnetometer calibration (hard iron + soft iron)
        float mx_cal = (mx - mx_offset) * mx_scale;
        float my_cal = (my - my_offset) * my_scale;
        float mz_cal = (mz - mz_offset) * mz_scale;

        // Remap axes for robot frame (Y+ toward USB, USB at back)
        // Robot forward = -Y_board, Robot right = -X_board
        float mx_robot = -my_cal;  // Robot forward component
        float my_robot = -mx_cal;  // Robot right component
        float mz_robot = mz_cal;   // Robot up component

        // Compute tilt-compensated heading from magnetometer
        // Roll/pitch from Madgwick also need remapping
        float rollRad = -pitch * DEG_TO_RAD;   // Robot roll = -board pitch
        float pitchRad = -roll * DEG_TO_RAD;   // Robot pitch = -board roll

        // Tilt compensation
        float cosRoll = cos(rollRad);
        float sinRoll = sin(rollRad);
        float cosPitch = cos(pitchRad);
        float sinPitch = sin(pitchRad);

        // Compensate magnetometer readings for tilt
        float mx_comp = mx_robot * cosPitch + mz_robot * sinPitch;
        float my_comp = mx_robot * sinRoll * sinPitch + my_robot * cosRoll - mz_robot * sinRoll * cosPitch;

        // Compute magnetic heading (with -10° correction for axis alignment)
        float magHeading = atan2(-my_comp, mx_comp) * RAD_TO_DEG - 10.0f;

        // Complementary filter: fuse gyro-integrated heading with magnetometer
        // Only trust magnetometer when robot is stationary (motors corrupt it heavily)
        static float fusedHeading = 0.0f;
        static bool initialized = false;
        static float lastMagHeading = 0.0f;

        if (!initialized) {
            fusedHeading = magHeading;
            lastMagHeading = magHeading;
            initialized = true;
        } else {
            // Gyro integration
            float dt = 1.0f / sensorRate;
            float gyroHeadingDelta = gz * dt;

            // Handle angle wrapping for fusion
            float diff = magHeading - fusedHeading;
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;

            // Detect motor activity by checking if gyro is showing significant rotation
            // or if magnetometer is jumping around (interference indicator)
            float magJump = fabs(magHeading - lastMagHeading);
            if (magJump > 180.0f) magJump = 360.0f - magJump;  // Handle wrap
            lastMagHeading = magHeading;

            // Detect activity: gyro rotating OR magnetometer jumping suspiciously
            bool motorsActive = (fabs(gz) > 5.0f) || (magJump > 10.0f);

            // Only apply magnetometer correction when stationary
            // When motors active: pure gyro integration (no mag correction)
            // When stationary: slow mag correction (0.5% per update)
            float magWeight = motorsActive ? 0.0f : 0.005f;

            fusedHeading = fusedHeading + gyroHeadingDelta + magWeight * diff;

            // Normalize
            while (fusedHeading > 180.0f) fusedHeading -= 360.0f;
            while (fusedHeading < -180.0f) fusedHeading += 360.0f;
        }

        heading = fusedHeading;
    }
}


float IMUInterface::getRoll() const {
    return roll - rollOffset;
}

float IMUInterface::getPitch() const {
    return pitch - pitchOffset;
}

float IMUInterface::getYaw() const {
    float yaw = heading - headingOffset;
    // Normalize to -180 to 180
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

namespace TOF {
    bool TOFSensors::initialize(const int* xshutPinsArray, const uint8_t* sensorAddressArray, uint8_t sda, uint8_t scl) {
        if (!xshutPinsArray || !sensorAddressArray) return configSet = initAttempted = lastInitSuccess = false;
        std::copy(xshutPinsArray, xshutPinsArray + SENSOR_COUNT, xshutPins);
        std::copy(sensorAddressArray, sensorAddressArray + SENSOR_COUNT, sensorAddresses);
        for (int i = 0; i < SENSOR_COUNT; ++i) { medianFilters[i].reset(); sensorOffsets[i] = 0.0f; }
        sdaPin = sda; sclPin = scl; configSet = true; initAttempted = false; lastInitSuccess = false;
        return true;
    }

    bool TOFSensors::begin() {

    if (!configSet) return initAttempted = lastInitSuccess = false;
    
    #if defined(ARDUINO_ARCH_SAMD)
        if (sdaPin != 0 || sclPin != 0) {
            Wire.begin(sdaPin, sclPin);
        } else {
            Wire.begin();
        }
    #else
        Wire.begin();
    #endif

        for (auto pin : xshutPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
        delay(100);
        bool allSensorsOk = true;
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            digitalWrite(xshutPins[i], HIGH); delay(50);
            VL53L0X& sensor = sensors[i]; sensor.setTimeout(500);
            if (!sensor.init()) { digitalWrite(xshutPins[i], LOW); allSensorsOk = false; continue; }
            sensor.setAddress(sensorAddresses[i]); sensor.setMeasurementTimingBudget(50000);
            medianFilters[i].reset(); sensorOffsets[i] = 0.0f;
        }
        initAttempted = true; lastInitSuccess = allSensorsOk;
        return allSensorsOk;
    }

    // MedianFilter Implementation
    MedianFilter::MedianFilter() : readIndex(0), numReadings(0) {
        std::fill(readings, readings + WINDOW_SIZE, 0.0f);
    }

    float MedianFilter::updateEstimate(float measurement) {
        readings[readIndex] = measurement; readIndex = (readIndex + 1) % WINDOW_SIZE;
        if (numReadings < WINDOW_SIZE) ++numReadings;
        float sorted[WINDOW_SIZE]; std::copy(readings, readings + numReadings, sorted);
        std::sort(sorted, sorted + numReadings);
        return sorted[numReadings / 2];
    }

    void MedianFilter::reset() { readIndex = numReadings = 0; }

    // TOFSensors Implementation
    uint16_t TOFSensors::readSensor(int sensorIndex) {
        return (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) ? sensors[sensorIndex].readRangeSingleMillimeters() : 0;
    }

    bool TOFSensors::sensorTimeout(int sensorIndex) {
        return (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) ? sensors[sensorIndex].timeoutOccurred() : true;
    }

    float TOFSensors::getFilteredDistance(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) return -1.0f;
        uint16_t raw = readSensor(sensorIndex);
        if (sensorTimeout(sensorIndex)) return -1.0f;
        return medianFilters[sensorIndex].updateEstimate((raw - sensorOffsets[sensorIndex]) / 10.0f);
    }

    void TOFSensors::resetFilter(int sensorIndex) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) medianFilters[sensorIndex].reset();
    }

    void TOFSensors::setSensorOffset(int sensorIndex, float offsetMM) {
        if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) sensorOffsets[sensorIndex] = offsetMM;
    }

    float TOFSensors::getSensorOffset(int sensorIndex) {
        return (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) ? sensorOffsets[sensorIndex] : 0.0f;
    }
}