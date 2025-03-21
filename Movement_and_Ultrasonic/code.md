        First Version (Ultrasonic wired with 8 pins in the front)

#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

// Initialize the LCD (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// Initialize the MPU-6050
MPU6050 mpu;

// Ultrasonic sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 10

void setup() {
  // Initialize the LCD
  lcd.begin(16, 2); // 16 columns, 2 rows
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Display static text
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Initialize MPU-6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while (1); // Stop if MPU-6050 is not connected
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}

void loop() {
  // Variables for ultrasonic sensor
  long duration;
  float distance;

  // Variables for MPU-6050
  int16_t ax, ay, az; // Accelerometer readings
  float angleX;

  // --- Ultrasonic Sensor Reading ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2; // Convert to cm

  // --- MPU-6050 Reading ---
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate tilt angle (in degrees) for X-axis
  angleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // --- Display on LCD ---
  // Display distance on the first row
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   "); // Clear extra characters

  // Display angle on the second row
  lcd.setCursor(0, 1);
  lcd.print("AngleX: ");
  lcd.print(angleX);
  lcd.print("   "); // Clear extra characters

  delay(500); // Update every 500ms
}


        
        
        Second Version, wiring redone to be 4 pins on the back

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

// Initialize the I2C LCD (address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the MPU-6050 sensor
MPU6050 mpu;

// Define ultrasonic sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 10

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects for M1 and M4 on the motor shield
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left motor on M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4); // Right motor on M4

// Forward declarations for motor control functions
void moveForward();
void stopMotors();

void setup() {
  // Initialize the I2C LCD with the correct dimensions
  lcd.begin(16, 2);
  lcd.backlight(); // Turn on the LCD backlight

  // Set up ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Display static text on the LCD
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Start I2C communication
  Wire.begin();

  // Initialize the MPU-6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while (1); // Halt operation if MPU-6050 fails to connect
  }

  // Initialize the motor shield (AFMS.begin() returns void)
  AFMS.begin();

  // Set initial motor speeds to 0
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  // Clear LCD and display new message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}

void loop() {
  // Variables for ultrasonic sensor
  long duration;
  float distance;

  // Variables for MPU-6050 readings
  int16_t ax, ay, az; // Accelerometer readings
  float angleX;

  // --- Ultrasonic Sensor Reading ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2; // Convert time to distance in centimeters

  // --- MPU-6050 Sensor Reading ---
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate the tilt angle (for the X-axis) in degrees
  angleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // --- Display Values on LCD ---
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm   "); // Extra spaces clear previous characters
  lcd.setCursor(0, 1);
  lcd.print("AngleX: ");
  lcd.print(angleX);
  lcd.print("   "); // Extra spaces clear previous characters

  // --- Movement Logic ---
  // Move forward if an object is between 20 and 50 cm away,
  // Otherwise, stop if the object is too close (<=20 cm) or too far (>50 cm)
  if (distance > 20 && distance <= 50) {
    moveForward();
  } else {
    stopMotors();
  }

  delay(100); // A short delay for stability
}

void moveForward() {
  // Set speed and run the motors for forward movement (reversed due to mounting)
  leftMotor->setSpeed(150); // Speed value (0-255)
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);  // Reverse button: 'BACKWARD' here makes it move forward by your mounting
  rightMotor->run(BACKWARD);
}

void stopMotors() {
  // Stop both motors from running
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}