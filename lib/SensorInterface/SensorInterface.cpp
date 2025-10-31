#include <SensorInterface.h>

IMUInterface::IMUInterface(float sampleRate) : sensorRate(sampleRate),
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

void IMUInterface::update() {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
    }
    bool hasMag = false;
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
        hasMag = true;
    }
    if (hasMag) {
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        filter.updateIMU(gx, gy, gz, ax, ay, az);
    }
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
}

float IMUInterface::getRoll() const {
    return roll;
}

float IMUInterface::getPitch() const {
    return pitch;
}

float IMUInterface::getYaw() const {
    return heading;
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
            sdaPin || sclPin ? Wire.begin(sdaPin, sclPin) : Wire.begin();
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