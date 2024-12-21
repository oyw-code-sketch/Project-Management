#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TRIG_PIN 11
#define ECHO_PIN 3

// Motor Pins
#define LEFT_MOTOR_IN1 10  
#define LEFT_MOTOR_IN2 13 
#define RIGHT_MOTOR_IN1 12 
#define RIGHT_MOTOR_IN2 2 

long distance = 0;
unsigned long distanceUpdateTime = 0;
const int distanceUpdateInterval = 100;

unsigned long movementStartTime = 0;
const unsigned long movementDuration = 1000;

enum MovementState { IDLE, MOVING_BACKWARD, SCANNING } movementState = IDLE;

// Initialize the LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  lcd.begin(16, 2);

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 connection failed");
    lcd.setCursor(0, 1);
    lcd.print("MPU Fail");
    while (1);
  }
}

void loop() {
  if (millis() - distanceUpdateTime >= distanceUpdateInterval) {
    distance = getDistance(TRIG_PIN, ECHO_PIN);
    distanceUpdateTime = millis();

    lcd.setCursor(0, 0);
    lcd.print("Dis:");
    lcd.print(distance);
    lcd.print("cm  ");
  }

  handleMovement();
}

void handleMovement() {
  switch (movementState) {
    case IDLE:
      if (distance > 10 && distance <= 60) {
        moveForward();
        movementState = IDLE;
      } else {
        movementState = MOVING_BACKWARD;
        movementStartTime = millis();
      }
      break;

    case MOVING_BACKWARD:
      moveBackward();
      if (millis() - movementStartTime >= movementDuration) {
        stopMotors();
        movementState = SCANNING;
      }
      break;

    case SCANNING:
      rotate360();
      if (distance > 10 && distance <= 60) {
        movementState = IDLE;
      } else {
        movementState = MOVING_BACKWARD;
        movementStartTime = millis();
      }
      break;
  }
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;
  return distance;
}

void rotate360() {
  sensors_event_t accel, gyro, temp;
  float totalRotation = 0.0;
  unsigned long lastTime = millis();

  while (totalRotation < 360.0) {
    mpu.getEvent(&accel, &gyro, &temp);
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;

    // Calculate rotation around Z-axis
    float zRotationRate = gyro.gyro.z * (180.0 / 3.14159265359);
    totalRotation += abs(zRotationRate) * deltaTime;

    lastTime = currentTime;

    // Update distance during rotation
    if (currentTime - distanceUpdateTime >= distanceUpdateInterval) {
      distance = getDistance(TRIG_PIN, ECHO_PIN);
      distanceUpdateTime = currentTime;

      lcd.setCursor(0, 0);
      lcd.print("Dis:");
      lcd.print(distance);
      lcd.print("cm  ");
    }

    // Check if an object is detected
    if (distance > 10 && distance <= 60) {
      stopMotors();
      handleMovement();
      return; // Exit the function to stop the 360-degree rotation
    }

    // Rotate the robot to the right
    moveRight();
  }

  // Stop the motors after completing the rotation
  stopMotors();
}

void moveRight() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}
