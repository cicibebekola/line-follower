#include <Arduino.h>
#include <QTRSensors.h>

// PID constants
float kp = 0.09;
float ki = 0.00001;
float kd = 0.2;
int P, I = 0, D, lastError = 0;  // I initialized to 0

// Motor speed limits
#define rightMaxSpeed 120
#define leftMaxSpeed 120
#define rightBaseSpeed 70
#define leftBaseSpeed 70

// DRV8871 motor driver pins
#define rightMotorDir 9  // Direction pin for right motor
#define rightMotorPWM 10  // PWM pin for right motor
#define leftMotorDir 3  // Direction pin for left motor
#define leftMotorPWM 5  // PWM pin for left motor

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Error threshold to prevent reverse motion for small errors
int errorThreshold = 900;

// Motor control function for DRV8871
void driveMotors(int rightSpeed , int leftSpeed) {
    // Right motor control
    if (rightSpeed >= 0) {
        digitalWrite(rightMotorDir, HIGH);  // Forward
    } else {
        digitalWrite(rightMotorDir, LOW);   // Reverse
        rightSpeed = -rightSpeed;  // Make speed positive for PWM
    }
    analogWrite(rightMotorPWM, rightSpeed);

    // Left motor control
    if (leftSpeed >= 0) {
        digitalWrite(leftMotorDir, HIGH);   // Forward
    } else {
        digitalWrite(leftMotorDir, LOW);    // Reverse
        leftSpeed = -leftSpeed;  // Make speed positive for PWM
    }
    analogWrite(leftMotorPWM, leftSpeed);
}

void wait() {
    driveMotors(0, 0);  // Stop both motors
}

void setup() {
    // Initialize Serial communication for logging
    Serial.begin(9600);

    // Sensor setup
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

    // Motor driver pin setup
    pinMode(rightMotorDir, OUTPUT);
    pinMode(rightMotorPWM, OUTPUT);
    pinMode(leftMotorDir, OUTPUT);
    pinMode(leftMotorPWM, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); 

    // Sensor calibration
    for (int i = 0; i < 100; i++) {
        qtr.calibrate();
        delay(10);
    }
    wait();  // Ensure motors are stopped during calibration
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000);
}

void loop() {
    uint16_t position = qtr.readLineWhite(sensorValues);

    // Error calculation based on new max error value
    int error = position - 3500;
    P = error;
    I = I + error;
    I = constrain(I, -10000, 10000);  // Prevent integral windup
    D = error - lastError;
    lastError = error;

    // Scale motor speed based on the new max error of 1200
    int motorSpeed = (P * kp + I * ki + D * kd) * abs(error) / 1200;

    // Adjust motor speeds based on PID values
    int rightMotorSpeed = rightBaseSpeed - motorSpeed;
    int leftMotorSpeed = leftBaseSpeed + motorSpeed;

    // Constrain motor speeds to avoid extreme values
    rightMotorSpeed = constrain(rightMotorSpeed, -rightMaxSpeed, rightMaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, -leftMaxSpeed, leftMaxSpeed);

    // Apply error threshold for smoother response to small errors
    if (abs(error) < errorThreshold) {
        rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);
        leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);
    }

    // Apply motor speed smoothing
    static int lastRightMotorSpeed = 0;
    static int lastLeftMotorSpeed = 0;

    rightMotorSpeed = (lastRightMotorSpeed + rightMotorSpeed) / 2;
    leftMotorSpeed = (lastLeftMotorSpeed + leftMotorSpeed) / 2;

    lastRightMotorSpeed = rightMotorSpeed;
    lastLeftMotorSpeed = leftMotorSpeed;

    // Drive motors with calculated speeds
    driveMotors(leftMotorSpeed, rightMotorSpeed);

    // Log error and motor speeds for debugging
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | Right Motor Speed: ");
    Serial.print(rightMotorSpeed);
    Serial.print(" | Left Motor Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.println();

    delay(20);  // Adjust delay if needed
}

