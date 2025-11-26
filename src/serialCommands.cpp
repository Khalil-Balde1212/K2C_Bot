#include "main.h"

// Extracted serial command processing from main.cpp
void serialCommands()
{
    if (Serial.available() <= 0) return;

    String input = Serial.readStringUntil('\n');
    input.trim();

    // Check for velocity control commands first (before switch statement)
    // to avoid conflict with 'v' speed command
    if (input.startsWith("velctl"))
    {
        String command = input.substring(7);
        command.trim();
        if (command == "on")
        {
            robot.enableVelocityControl(true);
        }
        else if (command == "off")
        {
            robot.enableVelocityControl(false);
        }
        else
        {
            Serial.println("Usage: velctl on/off");
        }
    }
    else if (input.startsWith("veltarget"))
    {
        // Parse: veltarget <vx> <vy> <omega>
        String params = input.substring(10);
        params.trim();
        
        int firstSpace = params.indexOf(' ');
        int secondSpace = params.indexOf(' ', firstSpace + 1);
        
        if (firstSpace > 0 && secondSpace > 0)
        {
            float vx = params.substring(0, firstSpace).toFloat();
            float vy = params.substring(firstSpace + 1, secondSpace).toFloat();
            float omega = params.substring(secondSpace + 1).toFloat();
            
            robot.setTargetVelocity(vx, vy, omega);
            Serial.print("Target velocity set: vx=");
            Serial.print(vx, 3);
            Serial.print(" m/s, vy=");
            Serial.print(vy, 3);
            Serial.print(" m/s, omega=");
            Serial.print(omega, 3);
            Serial.println(" rad/s");
        }
        else
        {
            Serial.println("Usage: veltarget <vx> <vy> <omega>");
        }
    }
    else if (input.startsWith("velgains"))
    {
        // Parse: velgains <kp> <ki> <kd> (applies to all three controllers)
        String params = input.substring(9);
        params.trim();
        
        int firstSpace = params.indexOf(' ');
        int secondSpace = params.indexOf(' ', firstSpace + 1);
        
        if (firstSpace > 0 && secondSpace > 0)
        {
            float kp = params.substring(0, firstSpace).toFloat();
            float ki = params.substring(firstSpace + 1, secondSpace).toFloat();
            float kd = params.substring(secondSpace + 1).toFloat();
            
            robot.setVelocityControlGains(kp, ki, kd, kp, ki, kd, kp, ki, kd);
            Serial.print("Velocity PID gains set: kp=");
            Serial.print(kp, 2);
            Serial.print(", ki=");
            Serial.print(ki, 2);
            Serial.print(", kd=");
            Serial.println(kd, 2);
        }
        else
        {
            Serial.println("Usage: velgains <kp> <ki> <kd>");
        }
    }
    else
    {
        // Standard character-based commands
        switch (input.charAt(0))
        {
            case 'h':
            {
                targetHeading = input.substring(1).toFloat();
                headingErrorIntegral = 0.0f;
                Serial.print("Target heading: ");
                Serial.println(targetHeading);
                break;
            }
            case 'v':
            {
                desiredSpeed = input.substring(1).toFloat();
                Serial.print("Speed: ");
                Serial.println(desiredSpeed);
                break;
            }
            case 'a':
            {
                char side = input.charAt(1);
                String angleStr = input.substring(2);
                float angle = angleStr.toFloat();
                
                if (side == 'l')
                {
                    Serial.print("Setting left pivot angle: ");
                    Serial.println(angle);
                    robot.setSteeringAngles(angle * PI / 180.0f, robot.getRightAngle());
                }
                else if (side == 'r')
                {
                    Serial.print("Setting right pivot angle: ");
                    Serial.println(angle);
                    robot.setSteeringAngles(robot.getLeftAngle(), angle * PI / 180.0f);
                } else {
                    angle = input.substring(1).toFloat();
                    Serial.println("Setting pivot angle: " + String(angle));
                    robot.setSteeringAngles(angle * PI / 180.0f, angle * PI / 180.0f);
                }
                break;
            }
            case 's':
            {
                char side = input.charAt(1);
                float speed = input.substring(2).toFloat();
                if (side == 'l')
                {
                    Serial.print("Setting left motor speed: ");
                    Serial.println(speed);
                    robot.setWheelSpeeds(speed, robot.getRightRPM());
                }
                else if (side == 'r')
                {
                    Serial.print("Setting right motor speed: ");
                    Serial.println(speed);
                    robot.setWheelSpeeds(robot.getLeftRPM(), speed);
                }
                break;
            }
            default:
            {
                if (input == "stop")
                {
                    desiredSpeed = 0.0f;
                    robot.stop();
                }
                else if (input == "reset")
                {
                    odometry.reset();
                    imu.calibrateOrientation();
                    imu.resetBiasEstimation();
                    robot.stop();
                    targetHeading = 0.0f;
                    headingErrorIntegral = 0.0f;
                    lastLeftCounts = *leftMotor.getCounts();
                    lastRightCounts = *rightMotor.getCounts();
                    Serial.println("Reset complete");
                }
                else if (input == "magcal")
                {
                    // Stop motors during calibration
                    robot.emergencyStop();
                    imu.calibrateMagnetometer(15); // 15 seconds to rotate robot
                    imu.calibrateOrientation();
                    Serial.println("Copy the calibration values above to setMagCalibration() in setup()");
                }
                else if (input == "pivotzero")
                {
                    robot.calibratePivotZero();
                }
                else if (input == "pivotreset")
                {
                    robot.resetPivotAngles();
                }
                else if (input == "quiet")
                {
                    quietMode = !quietMode;
                    Serial.print("Quiet mode: ");
                    Serial.println(quietMode ? "ON" : "OFF");
                }
                else if (input.startsWith("demo"))
                {
                    String command = input.substring(5);
                    command.trim();
                        if (command == "")
                        {
                            // Start the main demo (sensor-based)
                            startMainDemo();
                        }
                        else if (command == "stop")
                        {
                            stopMainDemo();
                        }
                        else if (command == "restart")
                        {
                            restartMainDemo();
                        }
                    else
                    {
                        Serial.println("Usage: demo [stop|restart]");
                    }
                }
                else if (input.startsWith("demo2"))
                {
                    String command = input.substring(6);
                    command.trim();
                    if (command == "")
                    {
                        startMainDemo();
                    }
                    else if (command == "stop")
                    {
                        stopMainDemo();
                    }
                    else
                    {
                        Serial.println("Usage: demo2 [stop]");
                    }
                }
                else if (input == "claw open")
                {
                    robot.setEndEffectorAngle(90.0f);
                    Serial.println("Claw opened");
                }
                else if (input == "claw close")
                {
                    robot.setEndEffectorAngle(0.0f);
                    Serial.println("Claw closed");
                }
                else if (input == "arm up")
                {
                    robot.setArmServosAngle(0.0f);
                    Serial.println("Arm moved up");
                }
                else if (input == "arm down")
                {
                    robot.setArmServosAngle(180.0f);
                    Serial.println("Arm moved down");
                }
                break;
            }
        }
    }
}
