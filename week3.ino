#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Initialize LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// MPU-6050
Adafruit_MPU6050 mpu;

// Pins for rotary encoders
#define CLK_A0 A0
#define CLK_A3 A3 

// Infrared sensors
#define IR_SENSOR_LEFT A2
#define IR_SENSOR_RIGHT A1

// Motor pins
#define MOTOR_PWM_LEFT 3
#define MOTOR_PWM_RIGHT 11
#define MOTOR_LEFT_PIN1 10
#define MOTOR_LEFT_PIN2 13
#define MOTOR_RIGHT_PIN1 12
#define MOTOR_RIGHT_PIN2 2

// Variables for encoders
volatile int counter_A0 = 0;
volatile int counter_A3 = 0;
int lastStateCLK_A0;
int lastStateCLK_A3;

float wheelDiameter = 6.2; // cm
float pulsesPerRevolution = 20.0;
float distancePerPulse = (3.14159265359 * wheelDiameter) / pulsesPerRevolution;
float distance_A0 = 0.0;
float distance_A3 = 0.0;

unsigned long moveStartTime = 0;  // Stores the time when forward movement starts
bool movingForward = false;       // Flag to track if the robot is in the 2-second movement phase
bool distanceTraveled = false;    // Flag to ensure path following after movement

// IR sensor thresholds
int leftIRThreshold = 200;  // Adjust this value based on your sensor's characteristics
int rightIRThreshold = 200 ;

void setup() {
  pinMode(CLK_A0, INPUT);
  pinMode(CLK_A3, INPUT);

  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  pinMode(MOTOR_PWM_LEFT, OUTPUT);
  pinMode(MOTOR_PWM_RIGHT, OUTPUT);

  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN2, OUTPUT);

  // Set the PWM frequency for ENA and ENB
  setPWMFrequency(MOTOR_PWM_LEFT, 1000);
  setPWMFrequency(MOTOR_PWM_RIGHT, 1000);

  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  lcd.clear();

  if (!mpu.begin()) {
    lcd.print("MPU6050 Error");
    while (1);
  }
  lcd.print("MPU6050 Ready");
  lcd.clear();

  lastStateCLK_A0 = digitalRead(CLK_A0);
  lastStateCLK_A3 = digitalRead(CLK_A3);
  moveStartTime = millis();
}

void loop() {
  static bool onRamp = false;          // Whether the robot is currently climbing the ramp
  static bool rampHandled = false;    // Whether the ramp detection has been handled
  static bool rampTopHandled = false; // Whether the top of the ramp logic has been handled
  static bool descending = false;     // Whether the robot is descending the ramp
  
  float avgAngle = calculateAverageAngle(); // Get the average ramp angle
  
  // Step 1: Follow the line using infrared sensors
  if (!onRamp && !rampHandled) {
    followPath(130,130);
    lcd.setCursor(0, 0);
    lcd.print("Angle: ");
    lcd.print(avgAngle, 1);
    lcd.print(" deg");

    if (avgAngle > 22) {  // Ramp detected
        stopMotorsWithTimeout(3000);  // Stop for 3 seconds
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Ramp Angle: ");
        lcd.print(avgAngle, 1);  // Display ramp angle
        onRamp = true;
        rampHandled = true;
    }
  }

  // Step 2: Climb the ramp and stop at the top
  if (onRamp && !rampTopHandled) {
    if (avgAngle < 10) {  // Reached the top of the ramp
        stopMotorsWithTimeout(4000);  // Pause for 4 second
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Top of Ramp");

        rotate360();  // Rotate 360 degrees
        stopMotorsWithTimeout(1000);  // Pause for 1 second after rotation

         // Move forward without using infrared sensors
        unsigned long moveStartTime = millis();
        while (millis() - moveStartTime < 1250) {  // Move for 1 seconds
            moveForward(150, 150);  // Move forward at speed 110
        } 
        stopMotorsWithTimeout(1000);  // Stop after moving forward
        // Reset distance counters for Step 3
        distance_A0 = 0;
        distance_A3 = 0;
        counter_A0 = 0;
        counter_A3 = 0;
        rampTopHandled = true;  // Mark top handled
        distanceTraveled = false;  // Reset flag for the next step
    } else {
        unsigned long moveStartTime = millis();
        while (millis() - moveStartTime < 500) {  // Move for 0.5 seconds
            moveForward(255, 255);  // Move forward at speed 110
        }
    }
  }

  /// Step 3: Follow the line for 100 cm, but only after the robot has moved down the ramp
if (rampTopHandled && !distanceTraveled) {
    // Start tracking distance only after the angle goes from negative to positive (indicating it moved down the ramp)
    if (avgAngle > 0) {  // Angle has returned to positive 
        distance_A0 = 0;  // Reset distance counters
        distance_A3 = 0;
        counter_A0 = 0;
        counter_A3 = 0;
        followPathForDistance(100.0,90,90);  // Follow the path for 100 cm
        stopMotorsWithTimeout(3000);  // Stop for 3 seconds
        distanceTraveled = true;  // Mark the distance has been traveled
    }
}
  // Step 4: Continue following the path until the end
  if (distanceTraveled) {
    followPathForDistance(1000.0 - (distance_A0 + distance_A3) / 2.0,90,90);  // Continue following the path
  }
}

// Custom stop function with a timeout
void stopMotorsWithTimeout(unsigned long durationMs) {
  unsigned long startTime = millis();
  stopMotors();
  while (millis() - startTime < durationMs) {
    // Perform other tasks if needed
  }
}

// Calculate average ramp angle
/*float calculateAverageAngle() {
  float angles[5] = {0};
  for (int i = 0; i < 5; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    angles[i] = atan2(accel.acceleration.x, accel.acceleration.z) * (180.0 / 3.14159265359);
  }
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += angles[i];
  }
  return sum / 5.0;
}*/
float calculateAverageAngle() {
  float angles[5] = {0};
  for (int i = 0; i < 5; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Assume gravitational acceleration is 9.8 m/s²
    float gravity = 9.8;

    // Calculate the angle of the X-axis with respect to gravity
    angles[i] = asin(accel.acceleration.x / gravity) * (180.0 / 3.14159265359);
  }

  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += angles[i];
  }
  return sum / 5.0;
}


// Follow the path using IR sensors
void followPath(int speedLeft, int speedRight) {
  int leftSensor = analogRead(IR_SENSOR_LEFT);
  int rightSensor = analogRead(IR_SENSOR_RIGHT);

  // Compare sensor readings to threshold for quicker response
  if (leftSensor > leftIRThreshold && rightSensor > rightIRThreshold) { 
    moveForward(speedLeft, speedRight); 
  } else if (rightSensor > rightIRThreshold && leftSensor <= leftIRThreshold) { 
    moveLeft(255, 255); 
  } else if (leftSensor > leftIRThreshold && rightSensor <= rightIRThreshold) {
    moveRight(255, 255); 
  } else {
    stopMotors(); 
  }
}

// Rotate 360 degrees
void rotate360() {
  sensors_event_t accel, gyro, temp;
  float totalRotation = 0.0; // Total accumulated rotation
  unsigned long lastTime = millis();

  lcd.clear();
  lcd.print("Calibrating...");
  lcd.clear();

  while (totalRotation < 360.0) {
    mpu.getEvent(&accel, &gyro, &temp);
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

    // Calculate net angular velocity (rad/s to deg/s conversion)
    float zRotationRate = gyro.gyro.z * (180.0 / 3.14159265359); // Z-axis rotation
    float netRotationRate = sqrt(pow(gyro.gyro.x, 2) + pow(gyro.gyro.y, 2) + pow(gyro.gyro.z, 2)) * (180.0 / 3.14159265359);

    // If rotation is primarily around Z-axis, update total rotation
    if (abs(zRotationRate) > abs(gyro.gyro.x) && abs(zRotationRate) > abs(gyro.gyro.y)) {
      totalRotation += abs(zRotationRate) * deltaTime;
    }

    lastTime = currentTime;

    // Rotate the robot to the right
    moveRight(220, 220);
  }

  // Stop the motors after completing the rotation
  stopMotors();

  // Display rotation completion
  lcd.clear();
  lcd.print("Rotation Done!");
}

// Follow the path for a set distance using IR sensors
void followPathForDistance(float targetDistance,int speedLeft, int speedRight) {

    // Record the starting time to calculate elapsed time
    moveStartTime = millis();
  
    while ((distance_A0 + distance_A3) / 2.0 < targetDistance) {
        followPath(speedLeft,speedRight); // Continue following the path using IR sensors

        // Encoder reading logic for A0
        int currentStateCLK_A0 = digitalRead(CLK_A0);
        if (currentStateCLK_A0 != lastStateCLK_A0 && currentStateCLK_A0 == 1) {
            counter_A0++;
            distance_A0 += distancePerPulse;
        }
        lastStateCLK_A0 = currentStateCLK_A0;

        // Encoder reading logic for A3
        int currentStateCLK_A3 = digitalRead(CLK_A3);
        if (currentStateCLK_A3 != lastStateCLK_A3 && currentStateCLK_A3 == 1) {
            counter_A3++;
            distance_A3 += distancePerPulse;
        }
        lastStateCLK_A3 = currentStateCLK_A3;

        // Update LCD with cumulative distance and time
        float averageDistance = (distance_A0 + distance_A3) / 2.0;
        lcd.setCursor(0, 0);
        lcd.print("Dis: ");
        lcd.print(averageDistance, 1);
        lcd.print(" cm");
        lcd.setCursor(0, 1);
        lcd.print("Time: ");
        unsigned long elapsedTime = millis() - moveStartTime;
        lcd.print(elapsedTime / 1000);  // Show seconds
        lcd.print(" s");
    }

    // Stop motors after reaching the target distance
    stopMotors();
}


// Move forward with a given speed
void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_PWM_LEFT, leftSpeed);
  analogWrite(MOTOR_PWM_RIGHT, rightSpeed);
  digitalWrite(MOTOR_LEFT_PIN1, HIGH);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
}

// Move left
void moveLeft(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_PWM_LEFT, leftSpeed);
  analogWrite(MOTOR_PWM_RIGHT, rightSpeed);
  digitalWrite(MOTOR_LEFT_PIN1, HIGH);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, LOW);
  digitalWrite(MOTOR_RIGHT_PIN2, HIGH);
}

// Move right
void moveRight(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_PWM_LEFT, leftSpeed);
  analogWrite(MOTOR_PWM_RIGHT, rightSpeed);
  digitalWrite(MOTOR_LEFT_PIN1, LOW);
  digitalWrite(MOTOR_LEFT_PIN2, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN1, HIGH);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
}

// Stop the motors
void stopMotors() {
  analogWrite(MOTOR_PWM_LEFT, 0);
  analogWrite(MOTOR_PWM_RIGHT, 0);
  digitalWrite(MOTOR_LEFT_PIN1, LOW);
  digitalWrite(MOTOR_LEFT_PIN2, LOW);
  digitalWrite(MOTOR_RIGHT_PIN1, LOW);
  digitalWrite(MOTOR_RIGHT_PIN2, LOW);
}

// Set PWM frequency
void setPWMFrequency(int pin, int freq) {
  analogWrite(pin, 0);
  analogWrite(pin, freq);  // Apply frequency value
}



