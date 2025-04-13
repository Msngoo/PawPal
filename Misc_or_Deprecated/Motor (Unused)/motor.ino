#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects for M1 and M2
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

void setup() {
    Serial.begin(9600);  // Start serial communication
    AFMS.begin();        // Initialize motor shield
    
    motorLeft->setSpeed(150);  // Set initial speed
    motorRight->setSpeed(150);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read command from Python
        command.trim(); // Remove any whitespace

        if (command == "FORWARD") {
            motorLeft->run(FORWARD);
            motorRight->run(FORWARD);
        } 
        else if (command == "BACKWARD") {
            motorLeft->run(BACKWARD);
            motorRight->run(BACKWARD);
        } 
        else if (command == "LEFT") {
            motorLeft->run(BACKWARD);
            motorRight->run(FORWARD);
        } 
        else if (command == "RIGHT") {
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
        } 
        else if (command == "STOP") {
            motorLeft->run(RELEASE);
            motorRight->run(RELEASE);
        }
        else if (command == "TURN") {
            motorLeft->setSpeed(100);
            motorRight->setSpeed(100);
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
        }
        else if (command == "SLOW_TURN") {
            motorLeft->setSpeed(70);
            motorRight->setSpeed(70);
            motorLeft->run(FORWARD);
            motorRight->run(BACKWARD);
        }
    }
}