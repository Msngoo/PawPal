#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

#define TRIG_PIN 11
#define ECHO_PIN 10

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

void moveForward(int speed);
void moveForwardSlow();
void stopMotors();
void turnLeft();
void turnRight();
void turnInPlace();
void turnSlow();

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  
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
  long duration;
  float distance;
  int16_t ax, ay, az;
  float angleX;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2;

  mpu.getAcceleration(&ax, &ay, &az);
  angleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   ");

  lcd.setCursor(0, 1);
  lcd.print("Angle: ");
  lcd.print(angleX);
  lcd.print("   ");

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "FORWARD") {
      moveForward(150);
    } else if (command == "LEFT") {
      turnLeft();
    } else if (command == "RIGHT") {
      turnRight();
    } else if (command == "TURN") {
      turnInPlace();
    } else if (command == "SLOW_TURN") {
      turnSlow();
    } else if (command == "STOP") {
      stopMotors();
    }
    Serial.print("CMD: ");
    Serial.println(command);
  } else {
    if (distance > 20 && distance <= 50) {
      moveForward(150);  // High torque
    } else if (distance > 5 && distance <= 20) {
      moveForward(100);  // Medium speed to maintain 20cm
    } else if (distance <= 5) {
      moveForwardSlow();
    } else {
      stopMotors();
    }
  }

  delay(100);
}

void moveForward(int speed) {
  leftMotor->setSpeed(speed);
  rightMotor->setSpeed(speed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void moveForwardSlow() {
  leftMotor->setSpeed(50);
  rightMotor->setSpeed(50);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stopMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void turnLeft() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(BACKWARD);
  delay(200);
  stopMotors();
}

void turnRight() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(RELEASE);
  delay(200);
  stopMotors();
}

void turnInPlace() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  delay(200);
  stopMotors();
}

void turnSlow() {
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(BACKWARD);
  delay(200);
  stopMotors();
}