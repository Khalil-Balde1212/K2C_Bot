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
    Serial.println("All calibration cleared.");
}

// Update the update() function to use bias correction:
void IMUInterface::update() {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        
        gx -= gx_bias;
        gy -= gy_bias;
        gz -= gz_bias;
        
        // REMOVE THE RADIAN CONVERSION - JUST USE DEGREES/SEC!
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
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