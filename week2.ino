#include <LiquidCrystal.h>

// Initialize the LiquidCrystal library
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define CLK_A4 A4
#define CLK_A5 A5

#define IR_SENSOR_LEFT A2  // Infrared sensor on analog pin A2
#define IR_SENSOR_RIGHT A1 // Infrared sensor on analog pin A1

#define ENA 3  // Enable pin for the left motor (PWM-capable)
#define ENB 11 // Enable pin for the right motor (PWM-capable)
#define MOTOR_LEFT_PIN1 2
#define MOTOR_LEFT_PIN2 13
#define MOTOR_RIGHT_PIN1 12
#define MOTOR_RIGHT_PIN2 A3

volatile int counter_A4 = 0; 
volatile int counter_A5 = 0; 
int lastStateCLK_A4;
int lastStateCLK_A5;

float wheelDiameter = 6.8; // Example: 6.8 cm
float pulsesPerRevolution = 20.0;
float distancePerPulse = (3.14159265359 * wheelDiameter) / pulsesPerRevolution;
float distance_A4 = 0.0; // Total distance for encoder A4
float distance_A5 = 0.0; // Total distance for encoder A5

void setup() {
  // Rotary encoders
  pinMode(CLK_A4, INPUT);
  pinMode(CLK_A5, INPUT);

  // Infrared sensors
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Motors
  pinMode(ENA, OUTPUT); // PWM-capable pins
  pinMode(ENB, OUTPUT); // PWM-capable pins
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN2, OUTPUT);

  // Set the PWM frequency for ENA and ENB
  setPWMFrequency(ENA, 1000); // 1 kHz
  setPWMFrequency(ENB, 1000);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Initial encoder states
  lastStateCLK_A4 = digitalRead(CLK_A4);
  lastStateCLK_A5 = digitalRead(CLK_A5);
}

void loop() {
  // Handle infrared sensors for line tracking
  int leftSensor = analogRead(IR_SENSOR_LEFT) > 512 ? HIGH : LOW;  
  int rightSensor = analogRead(IR_SENSOR_RIGHT) > 512 ? HIGH : LOW; 

  if (leftSensor == HIGH && rightSensor == HIGH) { 
    moveForward(110, 110); 
  } else if (rightSensor == LOW && leftSensor == HIGH) { 
    moveRight(255, 255); 
  } else if (leftSensor == LOW && rightSensor == HIGH) {
    moveLeft(255, 255); 
  } else {
    stopMotors(); 
  }

  // Handle encoder pulse counting
  int currentStateCLK_A4 = digitalRead(CLK_A4);
  if (currentStateCLK_A4 != lastStateCLK_A4 && currentStateCLK_A4 == 1) {
    counter_A4++;
    distance_A4 += distancePerPulse; 
  }
  lastStateCLK_A4 = currentStateCLK_A4;

  int currentStateCLK_A5 = digitalRead(CLK_A5);
  if (currentStateCLK_A5 != lastStateCLK_A5 && currentStateCLK_A5 == 1) {
    counter_A5++;
    distance_A5 += distancePerPulse; 
  }
  lastStateCLK_A5 = currentStateCLK_A5;

  // Calculate average distance
  float averageDistance = (distance_A4 + distance_A5) / 2.0;

  // Display average distance on LCD
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(averageDistance, 1); // Display average distance with one decimal place
  lcd.print(" cm");

  delay(10); 
}

void setPWMFrequency(int pin, int freq) {
  if (pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x03; // Set prescaler to 32 for Timer2
  }
}

// Function to move motors forward
void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_PIN1, HIGH);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
  analogWrite(ENA, leftSpeed); 
  analogWrite(ENB, rightSpeed); 
}

// Function to move motors left (turn right)
void moveLeft(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_PIN1, LOW);
  digitalWrite(MOTOR_LEFT_PIN2, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN1, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
  analogWrite(ENA, leftSpeed); 
  analogWrite(ENB, rightSpeed); 
}

// Function to move motors right (turn left)
void moveRight(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_PIN1, HIGH);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, LOW);
  digitalWrite(MOTOR_RIGHT_PIN2, HIGH);
  analogWrite(ENA, leftSpeed); 
  analogWrite(ENB, rightSpeed); 
}

// Function to stop motors
void stopMotors() {
  digitalWrite(MOTOR_LEFT_PIN1, LOW);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, LOW);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0); 
}