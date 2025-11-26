#ifndef SENSORINTERFACE_H
#define SENSORINTERFACE_H

//Libraries
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <algorithm>


class IMUInterface {
private:
    // IMU state
    bool initialized = false;
    unsigned long lastUpdateTime = 0;
    float dt = 0.0f;

    // Sensor readings
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    // Quaternion orientation
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    // Calibration offsets
    float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;
    float accel_offset_x = 0.0f, accel_offset_y = 0.0f, accel_offset_z = 0.0f;
    float mag_offset_x = 0.0f, mag_offset_y = 0.0f, mag_offset_z = 0.0f;

    // Calibration scales
    float accel_scale_x = 1.0f, accel_scale_y = 1.0f, accel_scale_z = 1.0f;
    float mag_scale_x = 1.0f, mag_scale_y = 1.0f, mag_scale_z = 1.0f;

    // Orientation offsets (for zeroing)
    float rollOffset = 0.0f;
    float pitchOffset = 0.0f;
    float headingOffset = 0.0f;

    // Madgwick filter parameters
    float beta = 0.1f; // Filter gain

    // Bias estimation for drift correction
    bool biasEstimationEnabled = false;
    float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
    float bias_alpha = 0.001f; // Bias learning rate (very slow)
    unsigned long bias_samples = 0;

    // Madgwick filter methods
    float invSqrt(float x);
    void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void quaternionToEuler(float& roll, float& pitch, float& yaw) const;

public:
    IMUInterface(float sampleRate = 104.0f);
    bool begin();
    void update();
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

    bool calibrateGyro(int samples = 100);
    bool calibrateMagnetometer(int durationSeconds = 10);
    void calibrateOrientation();
    void resetCalibration();

    // Set calibration values (use after running magcal once and noting the values)
    void setMagCalibration(float mx_off, float my_off, float mz_off,
                           float mx_sc, float my_sc, float mz_sc);

    // Dynamic filter tuning
    void setMadgwickBeta(float b) { beta = b; }
    float getMadgwickBeta() const { return beta; }

    // Continuous bias estimation and correction
    void enableBiasEstimation(bool enable = true) { biasEstimationEnabled = enable; }
    void updateBiasEstimation();
    void resetBiasEstimation();

    // Debug: get raw magnetometer values
    float getMx() const { return mx; }
    float getMy() const { return my; }
    float getMz() const { return mz; }
    
    // Get acceleration values (for velocity estimation)
    float getAccelX() const { return ax; }
    float getAccelY() const { return ay; }
    float getAccelZ() const { return az; }

    // Additional methods for compatibility
    float getHeading() const { return getYaw(); }
    void reset();
};

namespace TOF{
    const int SENSOR_COUNT = 2;
    class MedianFilter {
        private:
            static const int WINDOW_SIZE = 5;
            float readings[WINDOW_SIZE];
            int readIndex;
            int numReadings;

        public:
            MedianFilter();
            float updateEstimate(float measurement);
            void reset();
    };
    
    class TOFSensors{
    private:
        VL53L0X sensors[SENSOR_COUNT];
        MedianFilter medianFilters[SENSOR_COUNT];
        float sensorOffsets[SENSOR_COUNT] = {0.0};
        int xshutPins[SENSOR_COUNT] = {0};
        uint8_t sensorAddresses[SENSOR_COUNT] = {0};
        uint8_t sdaPin = 0;
        uint8_t sclPin = 0;
        bool configSet = false;
        bool initAttempted = false;
        bool lastInitSuccess = false;

    public:
        bool initialize(const int* xshutPinsArray, const uint8_t* sensorAddressArray,
                        uint8_t sda, uint8_t scl);

        bool begin();

        uint16_t readSensor(int sensorIndex);
        bool sensorTimeout(int sensorIndex);
        float getFilteredDistance(int sensorIndex); //<- distance in cm
        void resetFilter(int sensorIndex);

        //Calibration method
        void setSensorOffset(int sensorIndex, float offsetMM);
        float getSensorOffset(int sensorIndex);

        VL53L0X& getSensor(int index) {return sensors[index];}
        bool isInit() const {return initAttempted;}
        bool initComplete() const {return lastInitSuccess;}
        bool isConfigured() const {return configSet;}

    };

}


#endif //SENSORINTERFACE_H