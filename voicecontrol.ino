#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Motor Pins
#define IN1 10
#define IN2 13
#define IN3 12
#define IN4 2

// LCD Setup
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Software Serial for Voice Commands
SoftwareSerial voiceSerial(11, 3); // RX, TX

String voiceCommand; // Variable to hold the received voice command

void setup() {
    Serial.begin(9600);  // Debugging
    voiceSerial.begin(9600); // For voice commands
    lcd.begin(16, 2);
    lcd.print("Waiting for Voice");
    
    // Motor Pins Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    if (voiceSerial.available() > 0) {
        voiceCommand = voiceSerial.readString();
        voiceCommand.trim();                 // Remove leading/trailing spaces
        voiceCommand.toLowerCase();          // Convert to lowercase
        Serial.println("Received: " + voiceCommand); // Debugging info

        if (voiceCommand == "move forward") {
            moveForward();
            lcd.clear();
            lcd.print("Moving Forward");
            delay(2000);
            stopMotors();
            lcd.print("Stopped");
        } else if (voiceCommand == "move backward") {
            moveBackward();
            lcd.clear();
            lcd.print("Moving Backward");
            delay(2000);
            stopMotors();
            lcd.print("Stopped");
        } else if (voiceCommand == "turn left") {
            turnLeft();
            lcd.clear();
            lcd.print("Turning Left");
            delay(2000);
            stopMotors();
            lcd.print("Stopped");
        } else if (voiceCommand == "turn right") {
            turnRight();
            lcd.clear();
            lcd.print("Turning Right");
            delay(2000);
            stopMotors();
            lcd.print("Stopped");
        } else if (voiceCommand == "stop") {
            stopMotors();
            lcd.clear();
            lcd.print("Stopped");
        } else {
            lcd.clear();
            lcd.print("Invalid Command");
        }
    }
}

// Motor Control Functions
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

