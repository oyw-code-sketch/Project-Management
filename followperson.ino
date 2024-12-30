#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050.h>

#define TRIG_PIN 11
#define ECHO_PIN 3

// Motor Pins
#define LEFT_MOTOR_IN1 10  
#define LEFT_MOTOR_IN2 13 
#define RIGHT_MOTOR_IN1 12 
#define RIGHT_MOTOR_IN2 2 

long distance = 0;
unsigned long distanceUpdateTime = 0; // Tracks the last update time
const int distanceUpdateInterval = 100; // Update distance every 100ms
unsigned long accelUpdateTime = 0;      // Tracks the last acceleration update time
const int accelUpdateInterval = 1000;   // Update acceleration every 500ms


unsigned long movementStartTime = 0; // Tracks when a movement started
unsigned long movementDuration = 1000; // Movement duration (2 seconds)
enum MovementState { IDLE, MOVING_BACKWARD, MOVING_RIGHT } movementState = IDLE;

// Initialize the LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

// Create an MPU6050 object
MPU6050 mpu;

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
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    lcd.setCursor(0, 1);
    lcd.print("MPU Fail");
    while (1); 
  }
}

void loop() {
  // Update distance every 100ms
  if (millis() - distanceUpdateTime >= distanceUpdateInterval) {
    distance = getDistance(TRIG_PIN, ECHO_PIN);
    distanceUpdateTime = millis();

    // Update only the distance part of the LCD
    lcd.setCursor(0, 0); // First row
    lcd.print("Dis:");
    lcd.print(distance);
    lcd.print("cm  "); // Clear remaining characters
  }

  // Update acceleration every 500ms
  if (millis() - accelUpdateTime >= accelUpdateInterval) {
    accelUpdateTime = millis();

    // Variables to store acceleration values
    int16_t ax, ay, az;
    float accelX, accelY, accelZ;

    // Get acceleration values from MPU6050
    mpu.getAcceleration(&ax, &ay, &az);

    // Convert raw values to m/s^2
    accelX = (float)ax / 16384.0 * 9.81; // Assuming ±2g range, sensitivity = 16384 LSB/g
    accelY = (float)ay / 16384.0 * 9.81;
    accelZ = (float)az / 16384.0 * 9.81;

    // Print acceleration values for debugging
    Serial.print("Accel X: ");
    Serial.print(accelX);
    Serial.print(" m/s^2, Accel Y: ");
    Serial.print(accelY);
    Serial.print(" m/s^2, Accel Z: ");
    Serial.println(accelZ);

    // Update only the acceleration part of the LCD
    lcd.setCursor(10, 0); // First row (X acceleration)
    lcd.print("X:");
    lcd.print(accelX, 1); // X acceleration with 1 decimal

    lcd.setCursor(0, 1); // Second row (Y and Z acceleration)
    lcd.print("Y:");
    lcd.print(accelY, 1);
    lcd.print(" Z:");
    lcd.print(accelZ, 1); // Z acceleration with 1 decimal
    lcd.print("    ");   // Clear remaining characters
  }

  // Handle motor behavior based on distance
  handleMovement();
}


void handleMovement() {
  static unsigned long stopStartTime = 0; // Tracks when the stop started
  static bool isStopping = false;        // Flag for the stopping phase

  // Handle the stopping phase
  if (isStopping) {
    stopMotors(); // Ensure motors are stopped
    if (millis() - stopStartTime >= 1000) { // Check if 1 second has passed
      isStopping = false; // End the stopping phase
    }
    return; // Exit the function to avoid other movements
  }

  // Handle other movement states
  if (movementState == IDLE && distance > 10 && distance < 70) {
    moveForward(); // Move forward if distance is in range
  } else if (movementState == IDLE) {
    movementState = MOVING_BACKWARD; // Start the backward movement
    movementStartTime = millis();   // Track when it started
    moveBackward();
  }

  if (movementState == MOVING_BACKWARD && millis() - movementStartTime >= movementDuration) {
    movementState = MOVING_RIGHT;   // Switch to turning right
    movementStartTime = millis();   // Reset the start time
    moveRight();
  } else if (movementState == MOVING_RIGHT && millis() - movementStartTime >= movementDuration) {
    stopMotors();                   // Stop the motors after turning right
    isStopping = true;              // Enter stopping phase
    stopStartTime = millis();       // Record when the stop started
    movementState = IDLE;           // Reset to idle state after stopping
  }
}

// Function to get the distance from an ultrasonic sensor
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

// Function to move the robot forward
void moveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// Function to move the robot backward
void moveBackward() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);  
  digitalWrite(LEFT_MOTOR_IN2, HIGH); 
  digitalWrite(RIGHT_MOTOR_IN1, LOW);  
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

// Function to move the robot to the right
void moveRight() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);  
  digitalWrite(LEFT_MOTOR_IN2, HIGH); 
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);  
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}


