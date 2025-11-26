#include "main.h"

void debugPrints(unsigned long currentTime)
{
    // Debug output at 10Hz (only if not in quiet mode)
    static unsigned long lastPrint = 0;
    if (!quietMode && Serial && currentTime - lastPrint > 1000)
    {
        lastPrint = currentTime;

        // Robot controller status
        robot.printStatus();

        // Optional: additional status steps left as comments in the code
        // Serial.print("leftPivot Counts:\t");
        // Serial.print(*leftPivot.getCounts());
        // Serial.print(" | rightPivot Counts:\t");
        // Serial.print(*rightPivot.getCounts());
        // Serial.print(" | leftMotor Counts:\t");
        // Serial.print(*leftMotor.getCounts());
        // Serial.print(" | rightMotor Counts:\t");
        // Serial.print(*rightMotor.getCounts());
        // Serial.println();
    }
}
