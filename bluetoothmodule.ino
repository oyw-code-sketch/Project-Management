#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// HC-06 Bluetooth module
SoftwareSerial Bluetooth(11, 3); // RX, TX

// Motor driver pins
#define IN1 10
#define IN2 13
#define IN3 12
#define IN4 2

// LCD Pins (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

char command;

void setup() {
    // Initialize serial communication
    Serial.begin(9600);            // Debugging
    Bluetooth.begin(9600);         // Bluetooth module baud rate

    // Initialize motor driver pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // LCD Initialization
    lcd.begin(16, 2);              // 16x2 LCD
    lcd.print("System Ready");
    delay(2000);
    lcd.clear();
    lcd.print("Waiting for");
    lcd.setCursor(0, 1);
    lcd.print("BT Commands...");

    // Stop motors initially
    stopMotors();
    Serial.println("System ready. Waiting for Bluetooth commands...");
}

// Function definitions
void goForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Move Forward");
    displayDirection("Forward");
}

void goBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Move Backward");
    displayDirection("Backward");
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Turning Left");
    displayDirection("Left");
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Turning Right");
    displayDirection("Right");
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
    displayDirection("Stopped");
}

// Function to display the direction on the LCD
void displayDirection(const char *direction) {
    lcd.clear();
    lcd.print("Direction: ");
    lcd.setCursor(0, 1);
    lcd.print(direction);
}

void loop() {
    // Check if a command is received from Bluetooth
    if (Bluetooth.available()) {
        command = Bluetooth.read(); // Read the command
        Serial.print("Raw Command: ");
        Serial.println(command); // Debugging

        // Trim extra characters (newline or carriage return) if sent
        if (command == '\n' || command == '\r') {
            return; // Ignore these characters
        }

        Serial.print("Processed Command: ");
        Serial.println(command);

        // Perform the corresponding action
        switch (command) {
            case 'F': case 'f': goForward(); break;  // Forward
            case 'B': case 'b': goBackward(); break; // Backward
            case 'L': case 'l': turnLeft(); break;   // Turn left
            case 'R': case 'r': turnRight(); break;  // Turn right
            case 'S': case 's': stopMotors(); break; // Stop
            default:
                Serial.println("Invalid command.");
                lcd.clear();
                lcd.print("Invalid Command");
                break;
        }
    }
}
