#include <LiquidCrystal.h>

// Initialize the LCD (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Ultrasonic Sensor Pins
#define TRIG_PIN 11
#define ECHO_PIN 3

// Motor Pins
#define LEFT_MOTOR_IN1 10  
#define LEFT_MOTOR_IN2 13 
#define RIGHT_MOTOR_IN1 12 
#define RIGHT_MOTOR_IN2 2 

// Distance Threshold
#define SAFE_DISTANCE 15  // Distance threshold to detect obstacles

long distance;

void setup() {
  // Motor pin setup
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Ultrasonic sensor pin setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LCD Initialization
  lcd.begin(16, 2);  // 16x2 LCD
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
  
  Serial.begin(9600);
}

void loop() {
  distance = measureDistance(TRIG_PIN, ECHO_PIN);
  printLCD(distance);  // Print distance on LCD
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= SAFE_DISTANCE) {
    Serial.println("Obstacle detected! Moving away.");
    lcd.setCursor(0, 1);
    lcd.print("Stopping       ");
    stopMoving();
    delay(500);

    lcd.setCursor(0, 1);
    lcd.print("Moving Backward");
    moveBackward();
    delay(500);

    lcd.setCursor(0, 1);
    lcd.print("Turning Right  ");
    turnRight();
    delay(700);

    stopMoving();
    delay(500);  // Optional stop to stabilize after turning
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Moving Forward ");
    moveForward();
    Serial.println("Moving forward");
  }
  delay(100);
}

// Function to measure distance
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration =  (echoPin, HIGH);
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// Function to move forward
void moveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// Function to move backward
void moveBackward() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

// Function to turn right
void turnRight() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

// Function to stop moving
void stopMoving() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// Function to print distance on the LCD
void printLCD(long distance) {
  lcd.setCursor(0, 0);  // Set cursor to first row
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm   ");  // Clear old values with spaces
}
