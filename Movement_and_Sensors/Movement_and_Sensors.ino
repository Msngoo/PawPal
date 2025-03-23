// Arduino Code:
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

#define TRIG_PIN 11
#define ECHO_PIN 10

// --- Added microphone pin definitions ---
#define MIC_BCLK 9      // Bit Clock for the microphone
#define MIC_DOUT 8      // Data Output for the microphone
#define MIC_LRCL 7      // Left/Right Clock for the microphone
// SEL is not connected (or left as per breakout configuration)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

// Updated motor speeds and rotation delay
const int FORWARD_SPEED = 200;  // Increased from 150
const int BACKWARD_SPEED = 200; // Increased from 150
const int ROTATION_SPEED = 160; // Increased from 100
const int ROTATION_DELAY = 2500; // Increased from 2000ms

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // --- Initialize microphone pins ---
  pinMode(MIC_BCLK, INPUT);
  pinMode(MIC_DOUT, INPUT);
  pinMode(MIC_LRCL, INPUT);
  
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while (1);
  }
  
  AFMS.begin();
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}

void loop() {
  float distance = measureDistance();
  
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   ");
  
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "FOLLOW") {
      followDog(distance);
    } else if (command == "ROTATE") {
      rotate360();
    } else if (command == "STOP") {
      stopMotors();
    } else if (command == "BARK") {  // New branch for bark detection
      barkDetected();
    }
  }
  
  delay(100);
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration * 0.034) / 2;
}

void followDog(float distance) {
  if (distance > 20) {
    moveForward();
  } else if (distance < 20) {
    moveBackward();
  } else {
    stopMotors();
  }
}

void rotate360() {
  leftMotor->setSpeed(ROTATION_SPEED);
  rightMotor->setSpeed(ROTATION_SPEED);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  delay(ROTATION_DELAY);  // Adjusted for more granular control
  stopMotors();
}

void moveForward() {
  leftMotor->setSpeed(FORWARD_SPEED);
  rightMotor->setSpeed(FORWARD_SPEED);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void moveBackward() {
  leftMotor->setSpeed(BACKWARD_SPEED);
  rightMotor->setSpeed(BACKWARD_SPEED);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void stopMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// New function to handle bark detection, updated to show microphone pin data
void barkDetected() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BARK DETECTED");
  lcd.setCursor(0, 1);
  // Displaying microphone pin configuration on the LCD (BCLK=9, DOUT=8, LRCL=7)
  lcd.print("Mic: BCLK=9 DOUT=8");
  delay(1000); // Display message for 1 second
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}