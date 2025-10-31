#include <SensorInterface.h>

IMUInterface imu(20.0);
TOF::TOFSensors tofSensors;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while(1);
    }
    Serial.println("IMU initialized successfully.");
    
    // Calibrate gyroscope
    imu.calibrateGyro(100);
    
    // Let filter stabilize
    Serial.println("Stabilizing filter...");
    for (int i = 0; i < 100; i++) {
        imu.update();
        delay(10);
    }
    
    // Zero orientation
    imu.calibrateOrientation();
    
    // Initialize TOF sensors
    const int xshutPins[] = {2, 3, 4, 5};  // Your pins
    const uint8_t addresses[] = {0x30, 0x31, 0x32, 0x33};
    tofSensors.initialize(xshutPins, addresses, 0, 0);
    
    if (tofSensors.begin()) {
        Serial.println("TOF sensors initialized successfully.");
    } else {
        Serial.println("TOF init failed!");
    }
    
    Serial.println("Ready!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    
    // Update at 20 Hz (every 50ms)
    if (now - lastUpdate >= 50) {
        lastUpdate = now;
        
        imu.update();
        
        Serial.print("Yaw: ");
        Serial.println(imu.getYaw(), 2);
    }
}