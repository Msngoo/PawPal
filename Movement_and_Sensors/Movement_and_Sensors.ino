#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <Adafruit_MotorShield.h>

// Initialize the I2C LCD (address 0x27, 16 columns, 2 rows)
// If your LCD uses a different address, update accordingly.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the MPU6050 sensor
MPU6050 mpu;

// Define ultrasonic sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 10

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create motor objects for M1 and M4 on the motor shield
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Left motor on M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4); // Right motor on M4

// Function declarations for motor control
void moveForward();
void stopMotors();
void turnLeft();
void turnRight();
void turnInPlace();
void turnSlow();

void setup() {
  // Start serial communication for commands from the Raspberry Pi
  Serial.begin(9600);

  // Initialize the I2C LCD correctly for Frank de Brabander's library
  lcd.init();         // Use init() instead of begin()
  lcd.backlight();

  // Set up ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Display an initialization message
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Start I2C for MPU6050 and initialize it
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Error!");
    while (1); // Halt if the MPU6050 fails to connect
  }

  // Initialize the motor shield
  AFMS.begin();
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

  // Clear LCD and display the header message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance & Angle");
}

void loop() {
  long duration;
  float distance;
  int16_t ax, ay, az;  // MPU6050 accelerometer readings
  float angleX;

  // --- Ultrasonic Sensor Reading ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the echo pulse width and convert to distance in centimeters
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
  lcd.print(" cm   ");  // Extra spaces clear previous characters

  lcd.setCursor(0, 1);
  lcd.print("Angle: ");
  lcd.print(angleX);
  lcd.print("   ");

  // --- Serial Command Check ---
  // If a command is received from the Raspberry Pi, execute it.
  // Otherwise, use the default sensor-based movement.
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "FORWARD") {
      moveForward();
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
    // Print the received command for debugging.
    Serial.print("CMD: ");
    Serial.println(command);
  } else {
    // Default movement logic: move forward if object is between 20 and 50 cm away; otherwise, stop.
    if (distance > 20 && distance <= 50) {
      moveForward();
    } else {
      stopMotors();
    }
  }

  delay(100); // Brief delay for stability
}

void moveForward() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  // Motors are wired so that running BACKWARD makes the robot move forward.
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stopMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void turnLeft() {
  // To turn left, stop the left motor and run the right motor.
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(BACKWARD);
  delay(200); // Turn for a fixed duration
  stopMotors();
}

void turnRight() {
  // To turn right, run the left motor and stop the right motor.
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(RELEASE);
  delay(200);
  stopMotors();
}

void turnInPlace() {
  // Rotate in place by running motors in opposite directions.
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  delay(200);
  stopMotors();
}

void turnSlow() {
  // Perform a slow turn by running one motor at a lower speed.
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(BACKWARD);
  delay(200);
  stopMotors();
}