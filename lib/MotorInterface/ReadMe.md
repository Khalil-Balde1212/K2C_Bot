
# MotorInterface Library

This library provides an interface for controlling DC motors with encoders using the custom motor control PCB designed for the K2C Robot Platform. 

## Overview

The MotorInterface library allows you to:
- Control DC motor speed and direction via PWM signals
- Track encoder counts for position feedback
- Calculate motor RPM in real-time
- Support motor direction inversion
- Monitor motor status via serial output

The library uses a shared PWM driver that can control multiple motors simultaneously, making it ideal for differential drive robots or multi-motor systems.

## Hardware Requirements

- Adafruit PCA9685 PWM Servo Driver
- Quadrature encoders (A/B channels)
- Arduino-compatible microcontroller with interrupt support

## Features

- **PWM Motor Control**: Set raw speed values from -4095 to 4095
- **Encoder Feedback**: Read encoder counts for closed-loop control
- **RPM Calculation**: Automatic speed calculation based on time delta
- **Direction Inversion**: Easily invert motor direction for mounting flexibility
- **Multi-Motor Support**: Static PWM driver shared across all motor instances
- **Status Monitoring**: Print speed, counts, and RPM to serial console

## Methods

### Constructor
```cpp
Motor(int motorPortA, int motorPortB, int encoderPortA, int encoderPortB)
```
Creates a Motor instance with specified PWM and encoder pins.

**Parameters:**
- `motorPortA`: PWM driver channel for motor direction A
- `motorPortB`: PWM driver channel for motor direction B
- `encoderPortA`: Digital pin for encoder channel A
- `encoderPortB`: Digital pin for encoder channel B

**Example:**
```cpp
Motor leftMotor(0, 1, 2, 3);
```

---

### begin() [Static]
```cpp
static void begin()
```
Initializes the shared PWM driver. Must be called once in `setup()` before using any motors.

**Example:**
```cpp
Motor::begin();
```

---

### setRawSpeed()
```cpp
int* setRawSpeed(int speed)
```
Sets the motor PWM speed. Positive values = forward, negative = reverse.

**Parameters:**
- `speed`: Raw PWM value (-4095 to 4095)

**Returns:** Pointer to current speed value

**Example:**
```cpp
leftMotor.setRawSpeed(2000);  // Forward
leftMotor.setRawSpeed(-1500); // Reverse
```

---

### invertMotor()
```cpp
Motor invertMotor(bool inverted = true)
```
Sets motor direction inversion. Useful when motors are mounted in opposite orientations.

**Parameters:**
- `inverted`: `true` to invert direction, `false` for normal (default: `true`)

**Returns:** Motor instance (for method chaining)

**Example:**
```cpp
rightMotor.invertMotor(true);
```

---

### invertEncoder()
```cpp
Motor invertEncoder(bool inverted = true)
```
Sets encoder direction inversion independently from motor direction.

**Parameters:**
- `inverted`: `true` to invert encoder direction, `false` for normal (default: `true`)

**Returns:** Motor instance (for method chaining)

**Example:**
```cpp
leftMotor.invertEncoder(true);
```

---

### setCPR()
```cpp
Motor setCPR(float countsPerRevolution)
```
Sets the counts per revolution (CPR) for the encoder. This is required for accurate RPM calculations.

**Parameters:**
- `countsPerRevolution`: Number of encoder counts per full wheel revolution

**Returns:** Motor instance (for method chaining)

**Example:**
```cpp
leftMotor.setCPR(1440.0);  // For a 360 CPR encoder with 4x decoding
```

---

### update()
```cpp
void update(unsigned long lastTime, unsigned long currentTime)
```
Updates motor control and calculates RPM based on encoder counts and time delta. Should be called periodically in the main loop.

**Parameters:**
- `lastTime`: Previous timestamp in milliseconds
- `currentTime`: Current timestamp in milliseconds

**Example:**
```cpp
void loop(){
// your code
unsigned long currentTime = millis();
leftMotor.update(lastTime, currentTime);
lastTime = currentTime;
//
```

---

### updateCounts()
```cpp
void updateCounts()
```
Updates encoder counts based on current encoder state. **Must be called from an interrupt handler** attached to encoder channel A.

**Example:**
```cpp
attachInterrupt(digitalPinToInterrupt(myMotor.encA), 
                []() { myMotor.updateCounts(); }, 
                RISING);
```

---

### getSpeed()
```cpp
int getSpeed() const
```
Returns the current raw speed value.

**Example:**
```cpp
int currentSpeed = leftMotor.getSpeed();
```

---

### getCounts()
```cpp
const int* getCounts() const
```
Returns a pointer to the current encoder count.

**Example:**
```cpp
int encoderCount = *leftMotor.getCounts();
```

---

### currentRPM()
```cpp
const float* currentRPM() const
```
Returns a pointer to the current RPM value.

**Example:**
```cpp
float rpm = *leftMotor.currentRPM();
```

---


### coast()

The `coast()` function disables active braking on the motor, allowing it to spin freely and gradually come to a stop due to friction and inertia. This is useful when you want the motor to stop without applying force, preserving momentum.

---

### brake()

The `brake()` function actively stops the motor by applying resistance, causing it to halt quickly. This is useful for precise stopping or when safety requires the motor to stop immediately.




---

### printStatus()
```cpp
void printStatus()
```
Prints current speed, encoder counts, and RPM to the serial monitor.

**Example:**
```cpp
leftMotor.printStatus();
```

---

### setVelocity()
```cpp
void setVelocity(double velocity)
```
WARNING: NOT IMPLEMENTED!!
Sets the desired velocity (implementation depends on wheel parameters).

**Example:**
```cpp
leftMotor.setVelocity(1.5);
```

---

## Usage Example

### Creating a motor object
In order to create a motor object you need to:
- Construct the object
- Attach encoder interrupts
- Invoke the static begin method ONCE (even for multiple instances)


And periodically invoke the update(last time, current time) method to update control

ex.

```cpp
Motor myMotor;
unsigned long lastTime = 0, currentTime = 0;

void setup(){
    myMotor = Motor(MotorPinA, MotorPinB, EncoderDIO1, EncoderDIO2); //create motors based on PWM Driver pin numbers, and encoder pins.
    //attach interrupt pin
    attachInterrupt(digitalPinToInterrupt(myMotor.encA), [](){myMotor.updateCounts(); }, RISING);
    

    Motor::begin(); //this only needs to be called once in order to initialize the PWM Servo Driver
}

void loop(){
    /*
    * Your code goes here
    */

   currentTime = millis();
   myMotor.update(lastTime, currentTime); //delta time is calculated in milliseconds
   lastTime = currentTime();
}
```

## Note from Khalil
As of October 31st, the current pinmaps for the motors and encoders are

Motor leftMotor(12, 13, 2, 7);
Motor rightMotor(7, 6, 10, 9);
Motor leftPivot(15, 14, 11, 8);
Motor rightPivot(5, 4, 3, 4);

leftMotor.setCPR(1440.0f).invertMotor(true);
rightMotor.setCPR(1440.0f);
leftPivot.setCPR(2200.0f).invertEncoder(true);
rightPivot.setCPR(2200.0f).invertEncoder(true);