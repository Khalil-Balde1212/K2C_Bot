#include <Arduino.h>
#include <MotorInterface.h>
#include <SensorInterface.h>

Motor leftMotor(12, 13, 2, 7);
Motor rightMotor(7, 6, 10, 9);
Motor leftPivot(15, 14, 11, 8);
Motor rightPivot(5, 4, 3, 4);


unsigned long lastTime = 0;
unsigned long lastProcessTime;

IMUInterface imu(20.0);
TOF::TOFSensors tofSensors;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Motor::begin();
    leftMotor.setCPR(1440.0f).invertMotor(true);
    rightMotor.setCPR(1440.0f);
    leftPivot.setCPR(2200.0f).invertEncoder(true);
    rightPivot.setCPR(2200.0f).invertEncoder(true);


    lastProcessTime = 0;



    attachInterrupt(digitalPinToInterrupt(leftMotor.encA), []()
                    { leftMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotor.encA), []()
                    { rightMotor.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(leftPivot.encA), []()
                    { leftPivot.updateCounts(); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightPivot.encA), []()
                    { rightPivot.updateCounts(); }, RISING);
    leftMotor.setRawSpeed(0);
    rightMotor.setRawSpeed(0);
    leftPivot.setRawSpeed(0);
    rightPivot.setRawSpeed(0);
            
    // Initialize IMU
    if (!imu.begin())
    {
        Serial.println("IMU init failed!");
        while (1);
    }
    Serial.println("IMU initialized successfully.");

    // Calibrate gyroscope
    imu.calibrateGyro(100);

    // Let filter stabilize
    Serial.println("Stabilizing filter...");
    for (int i = 0; i < 100; i++)
    {
        imu.update();
        delay(10);
    }

    // Zero orientation
    imu.calibrateOrientation();

    // Initialize TOF sensors
    const int xshutPins[] = {5, 6, 12, 13}; // Your pins
    const uint8_t addresses[] = {0x30, 0x31, 0x32, 0x33};
    tofSensors.initialize(xshutPins, addresses, 0, 0);

    if (tofSensors.begin())
    {
        Serial.println("TOF sensors initialized successfully.");
    }
    else
    {
        Serial.println("TOF init failed!");
    }

    Serial.println("Ready!");
}

double targetHeading = 0;


void loop()
{
    //lol
    // double error = targetHeading - imu.getYaw(); // Example: using yaw as error
    // leftMotor.setRawSpeed(-error*500);
    // rightMotor.setRawSpeed(error*500);

    rightPivot.setRawSpeed(4096);

    leftMotor.update(lastProcessTime, millis());
    rightMotor.update(lastProcessTime, millis());
    leftPivot.update(lastProcessTime, millis());
    rightPivot.update(lastProcessTime, millis());
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        double newHeading = input.toDouble();
        if (!isnan(newHeading)) {
            targetHeading = newHeading;
            Serial.print("New target heading set to: ");
            Serial.println(targetHeading, 2);
        }
    }

    lastProcessTime = millis();
    if (millis() - lastTime > 50)
    {
        lastTime = millis();

        imu.update();

        // Serial.print("Yaw: ");
        // Serial.println(imu.getYaw(), 2);

        // Update TOF sensors
        leftPivot.printStatus();
    }
}