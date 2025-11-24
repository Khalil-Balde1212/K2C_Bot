#ifndef SENSORINTERFACE_H
#define SENSORINTERFACE_H

//Libraries
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <algorithm>


class IMUInterface {
private:

    float sensorRate; 
    Madgwick filter;
    
    // IMU variables
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    
    // Orientation values
    float roll = 0.0f, pitch = 0.0f, heading = 0.0f;

    float rollOffset = 0.0f;
    float pitchOffset = 0.0f;
    float headingOffset = 0.0f;
    
    float gx_bias = 0.0f;  // Gyro bias
    float gy_bias = 0.0f;
    float gz_bias = 0.0f;

    // Magnetometer hard iron offsets
    float mx_offset = 0.0f;
    float my_offset = 0.0f;
    float mz_offset = 0.0f;

    // Magnetometer soft iron scale factors
    float mx_scale = 1.0f;
    float my_scale = 1.0f;
    float mz_scale = 1.0f;

public:
    IMUInterface(float sampleRate = 104.00);
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

    // Debug: get raw magnetometer values
    float getMx() const { return mx; }
    float getMy() const { return my; }
    float getMz() const { return mz; }
};

namespace TOF{
    const int SENSOR_COUNT = 3;
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