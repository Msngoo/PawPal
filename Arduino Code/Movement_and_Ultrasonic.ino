#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

// Initialize the I2C LCD (address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the MPU6050 sensor
MPU6050 mpu;

// Define ultrasonic sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 10

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects for M1 and M4 on the motor shield
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left motor on M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4); // Right motor on M4

// Function declarations for motor control
void moveForward();
void stopMotors();

void setup() {
  // Initialize the I2C LCD with correct dimensions and turn on backlight
  lcd.begin(16, 2);
  lcd.backlight();

  // Set up the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Display an initialization message
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Start I2C communication for both the LCD and MPU6050
  Wire.begin();

  // Initialize the MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while (1); // Halt if the MPU6050 fails to connect
  }

  // Initialize the motor shield
  AFMS.begin();

  // Set initial motor speeds to 0
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  // Clear LCD and display a new header message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}

void loop() {
  long duration;
  float distance;

  int16_t ax, ay, az;  // Accelerometer readings from MPU6050
  float angleX;

  // --- Ultrasonic Sensor Reading ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the echo pulse width and convert to distance (cm)
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2;

  // --- MPU6050 Reading (Accelerometer Data) ---
  mpu.getAcceleration(&ax, &ay, &az);
  // Calculate a tilt angle (around the X-axis) in degrees
  angleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // --- Display the values on the LCD ---
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   ");  // Extra spaces help clear any old characters

  lcd.setCursor(0, 1);
  lcd.print("AngleX: ");
  lcd.print(angleX);
  lcd.print("   ");

  // --- Movement Logic ---
  // Move forward if the object is between 20 and 50 cm away;
  // otherwise, stop if it is too close (<=20 cm) or too far (>50 cm)
  if (distance > 20 && distance <= 50) {
    moveForward();
  } else {
    stopMotors();
  }

  delay(100); // Brief delay for stability
}

void moveForward() {
  // Set the motor speed and run the motors for forward movement.
  // Note: The motors are wired so that running "BACKWARD" makes the robot move forward.
  leftMotor->setSpeed(150);  // Speed value: 0-255
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stopMotors() {
  // Stop both motors
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}