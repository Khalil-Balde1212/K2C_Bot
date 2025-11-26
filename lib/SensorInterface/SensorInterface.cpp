#include <SensorInterface.h>

IMUInterface::IMUInterface(float sampleRate)
    : initialized(false), lastUpdateTime(0), dt(0.0f),
      ax(0.0f), ay(0.0f), az(0.0f),
      gx(0.0f), gy(0.0f), gz(0.0f),
      mx(0.0f), my(0.0f), mz(0.0f),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      gyro_offset_x(0.0f), gyro_offset_y(0.0f), gyro_offset_z(0.0f),
      accel_offset_x(0.0f), accel_offset_y(0.0f), accel_offset_z(0.0f),
      mag_offset_x(0.0f), mag_offset_y(0.0f), mag_offset_z(0.0f),
      accel_scale_x(1.0f), accel_scale_y(1.0f), accel_scale_z(1.0f),
      mag_scale_x(1.0f), mag_scale_y(1.0f), mag_scale_z(1.0f),
      rollOffset(0.0f), pitchOffset(0.0f), headingOffset(0.0f),
      beta(0.04f), biasEstimationEnabled(false),
      gyro_bias_x(0.0f), gyro_bias_y(0.0f), gyro_bias_z(0.0f),
      bias_alpha(0.001f), bias_samples(0) {
}

bool IMUInterface::begin() {
    if (!IMU.begin()) {
        initialized = false;
        return false;
    }

    initialized = true;
    lastUpdateTime = millis();

    // Initialize quaternion (no rotation)
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

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
    
    gyro_offset_x = sum_gx / samples;
    gyro_offset_y = sum_gy / samples;
    gyro_offset_z = sum_gz / samples;
    
    Serial.println();
    Serial.print("Gyro bias: gx=");
    Serial.print(gyro_offset_x);
    Serial.print(" gy=");
    Serial.print(gyro_offset_y);
    Serial.print(" gz=");
    Serial.println(gyro_offset_z);
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
    mag_offset_x = (mx_max + mx_min) / 2.0f;
    mag_offset_y = (my_max + my_min) / 2.0f;
    mag_offset_z = (mz_max + mz_min) / 2.0f;

    // Compute soft iron scale factors (normalize to sphere)
    float mx_range = (mx_max - mx_min) / 2.0f;
    float my_range = (my_max - my_min) / 2.0f;
    float mz_range = (mz_max - mz_min) / 2.0f;
    float avg_range = (mx_range + my_range + mz_range) / 3.0f;

    if (mx_range > 0) mag_scale_x = avg_range / mx_range;
    if (my_range > 0) mag_scale_y = avg_range / my_range;
    if (mz_range > 0) mag_scale_z = avg_range / mz_range;

    Serial.println("\n=== Magnetometer Calibration Complete ===");
    Serial.print("Hard iron offsets: X=");
    Serial.print(mag_offset_x, 2);
    Serial.print(" Y=");
    Serial.print(mag_offset_y, 2);
    Serial.print(" Z=");
    Serial.println(mag_offset_z, 2);
    Serial.print("Soft iron scales:  X=");
    Serial.print(mag_scale_x, 3);
    Serial.print(" Y=");
    Serial.print(mag_scale_y, 3);
    Serial.print(" Z=");
    Serial.println(mag_scale_z, 3);
    Serial.print("Total samples: ");
    Serial.println(samples);

    return samples > 50;  // Need reasonable number of samples
}

void IMUInterface::calibrateOrientation() {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    rollOffset = roll * RAD_TO_DEG;
    pitchOffset = pitch * RAD_TO_DEG;
    headingOffset = yaw * RAD_TO_DEG;
    Serial.println("Orientation zeroed at current position.");
}

void IMUInterface::resetCalibration() {
    gyro_offset_x = 0.0f;
    gyro_offset_y = 0.0f;
    gyro_offset_z = 0.0f;
    rollOffset = 0.0f;
    pitchOffset = 0.0f;
    headingOffset = 0.0f;
    mag_offset_x = 0.0f;
    mag_offset_y = 0.0f;
    mag_offset_z = 0.0f;
    mag_scale_x = 1.0f;
    mag_scale_y = 1.0f;
    mag_scale_z = 1.0f;
    Serial.println("All calibration cleared.");
}

void IMUInterface::setMagCalibration(float mx_off, float my_off, float mz_off,
                                      float mx_sc, float my_sc, float mz_sc) {
    mag_offset_x = mx_off;
    mag_offset_y = my_off;
    mag_offset_z = mz_off;
    mag_scale_x = mx_sc;
    mag_scale_y = my_sc;
    mag_scale_z = mz_sc;

    Serial.println("Magnetometer calibration set:");
    Serial.print("  Hard iron: X=");
    Serial.print(mag_offset_x, 2);
    Serial.print(" Y=");
    Serial.print(mag_offset_y, 2);
    Serial.print(" Z=");
    Serial.println(mag_offset_z, 2);
    Serial.print("  Soft iron: X=");
    Serial.print(mag_scale_x, 3);
    Serial.print(" Y=");
    Serial.print(mag_scale_y, 3);
    Serial.print(" Z=");
    Serial.println(mag_scale_z, 3);
}

void IMUInterface::updateBiasEstimation() {
    if (!biasEstimationEnabled) return;

    // Only update bias when the robot is relatively still (low angular velocity)
    float gyro_magnitude = sqrt(gx * gx + gy * gy + gz * gz);
    if (gyro_magnitude < 0.1f) { // Less than ~6 deg/s
        // Update bias estimates using exponential moving average
        gyro_bias_x = (1.0f - bias_alpha) * gyro_bias_x + bias_alpha * gx;
        gyro_bias_y = (1.0f - bias_alpha) * gyro_bias_y + bias_alpha * gy;
        gyro_bias_z = (1.0f - bias_alpha) * gyro_bias_z + bias_alpha * gz;
        bias_samples++;
    }
}

void IMUInterface::resetBiasEstimation() {
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    bias_samples = 0;
}

void IMUInterface::update() {
    if (!initialized) return;

    unsigned long currentTime = millis();
    dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;

    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f; // Default to 10ms if timing is off
    }

    // Read sensor data
    bool accelAvailable = IMU.accelerationAvailable();
    bool gyroAvailable = IMU.gyroscopeAvailable();
    bool magAvailable = IMU.magneticFieldAvailable();

    if (accelAvailable) {
        IMU.readAcceleration(ax, ay, az);
        // Apply calibration
        ax = (ax - accel_offset_x) * accel_scale_x;
        ay = (ay - accel_offset_y) * accel_scale_y;
        az = (az - accel_offset_z) * accel_scale_z;
    }

    if (gyroAvailable) {
        IMU.readGyroscope(gx, gy, gz);
        // Convert to radians/second and apply calibration
        gx = (gx - gyro_offset_x) * DEG_TO_RAD;
        gy = (gy - gyro_offset_y) * DEG_TO_RAD;
        gz = (gz - gyro_offset_z) * DEG_TO_RAD;

        // Apply continuous bias correction if enabled
        if (biasEstimationEnabled && bias_samples > 100) {
            gx -= gyro_bias_x;
            gy -= gyro_bias_y;
            gz -= gyro_bias_z;
        }
    }

    if (magAvailable) {
        IMU.readMagneticField(mx, my, mz);
        // Apply calibration
        mx = (mx - mag_offset_x) * mag_scale_x;
        my = (my - mag_offset_y) * mag_scale_y;
        mz = (mz - mag_offset_z) * mag_scale_z;
    }

    // Run Madgwick filter
    if (accelAvailable && gyroAvailable) {
        if (magAvailable) {
            // 9DOF Madgwick filter (with magnetometer)
            madgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        } else {
            // 6DOF Madgwick filter (IMU only)
            madgwickUpdateIMU(gx, gy, gz, ax, ay, az);
        }
    }

    // Update bias estimation if enabled
    updateBiasEstimation();
}


float IMUInterface::getRoll() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return roll * RAD_TO_DEG - rollOffset;
}

float IMUInterface::getPitch() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    return pitch * RAD_TO_DEG - pitchOffset;
}

float IMUInterface::getYaw() const {
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    float yaw_deg = yaw * RAD_TO_DEG - headingOffset;
    // Normalize to -180 to 180
    while (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    return yaw_deg;
}

void IMUInterface::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    lastUpdateTime = millis();
}

// Fast inverse square root approximation
float IMUInterface::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Madgwick 9DOF filter (with magnetometer)
void IMUInterface::madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Madgwick 6DOF filter (IMU only, no magnetometer)
void IMUInterface::madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Convert quaternion to Euler angles
void IMUInterface::quaternionToEuler(float& roll, float& pitch, float& yaw) const {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);
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