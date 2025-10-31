#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>

Motor motor(12, 13, 2, 7);
int i = -4095;
bool up = true;
unsigned long lastTime = 0;
unsigned long lastProcessTime;


IMUInterface imu(20.0);
TOF::TOFSensors tofSensors;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
  
    Motor::begin();
    motor.CountsPerRevolution = 1400; // Set according to your encoder specification
    lastProcessTime = 0;
    attachInterrupt(digitalPinToInterrupt(motor.encA), [](){ motor.updateCounts();}, RISING);
  
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
  if(up) i++;
  else i--;
  if(i >= 4095) up = false;
  if(i <= -4095) up = true;

  motor.setRawSpeed(up? -4096: 4096);

  
  motor.update(lastProcessTime, millis());
  lastProcessTime = millis();

  if(millis() - lastTime > 50){
    lastTime = millis();
    
    imu.update();
        
    Serial.print("Yaw: ");
    Serial.println(imu.getYaw(), 2);
    //motor.printStatus();
  }
}