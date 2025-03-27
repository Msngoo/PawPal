#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

// Initialize the LCD (I2C address 0x27, 16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

#define TRIG_PIN 11
#define ECHO_PIN 10

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

// Motor speed and rotation settings.
const int FORWARD_SPEED = 200;  
const int BACKWARD_SPEED = 200; 
const int ROTATION_SPEED = 160; 
const int ROTATION_DELAY = 2500; // in milliseconds

// Global treat counter.
int treatCount = 0;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while(1);
  }
  
  AFMS.begin();
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

void loop() {
  float distance = measureDistance();
  
  // Update LCD: first line displays distance, second line shows treat count.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm");
  lcd.setCursor(0, 1);
  lcd.print("Treats: ");
  lcd.print(treatCount);
  
  // Process incoming serial commands from the Pi.
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "FOLLOW") {
      followDog(distance);
    } else if (command == "ROTATE") {
      rotate360();
    } else if (command == "STOP") {
      stopMotors();
    } else if (command == "BARK") {
      barkDetected();
    } else if (command == "TREAT") {
      // When the Pi fires the solenoid, it sends "TREAT"
      treatCount++;
      // Optionally, refresh the LCD immediately:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(distance);
      lcd.print(" cm");
      lcd.setCursor(0, 1);
      lcd.print("Treats: ");
      lcd.print(treatCount);
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
  delay(ROTATION_DELAY);
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

void barkDetected() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BARK DETECTED");
  delay(10000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(measureDistance());
  lcd.print(" cm");
  lcd.setCursor(0, 1);
  lcd.print("Treats: ");
  lcd.print(treatCount);
}