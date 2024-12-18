#include <SoftwareSerial.h>

// Motor driver pins
#define IN1 10
#define IN2 13
#define IN3 12
#define IN4 2

// Ultrasonic sensor pins
const int trigPin = 11;
const int echoPin = 3;

// LED and buzzer pins
const int greenLEDPin = 5;
const int redLEDPin = 6;
const int buzzerPin = 7;

// Bluetooth module pins
SoftwareSerial Bluetooth(4, 9); // RX, TX

char command;
unsigned long previousMillis = 0;    // Timer for non-blocking delays
const long buzzerInterval = 500;    // 500 ms interval for buzzer

bool buzzerState = false;

void setup() {
    Serial.begin(9600);           // Debugging
    Bluetooth.begin(9600);        // Bluetooth module baud rate

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(greenLEDPin, OUTPUT);
    pinMode(redLEDPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);

    stopMotors();
    Serial.println("System ready. Waiting for Bluetooth commands...");
}

float measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2; // Convert to cm
}

void goForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving forward...");
}

void goBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving backward...");
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Turning left...");
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Turning right...");
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Motors stopped.");
}
void parkingSensor() {
    float distance = measureDistance();  // Get the current distance
    Serial.print("Measured Distance: ");
    Serial.print(distance);
    Serial.println(" cm"); // Print the distance in centimeters

    unsigned long currentMillis = millis();

    if (distance > 12 && distance <= 50) {
        // If object is within 10-50 cm
        digitalWrite(greenLEDPin, HIGH);  // Turn on Green LED
        digitalWrite(redLEDPin, LOW);     // Turn off Red LED
        noTone(buzzerPin);                // Ensure buzzer is off
        buzzerState = false;              // Reset buzzer state
    } else if (distance > 7 && distance <= 12) {
        // If object is closer than 5-10 cm
        digitalWrite(greenLEDPin, LOW);   // Turn off Green LED

        if (currentMillis - previousMillis >= buzzerInterval) {
            previousMillis = currentMillis;
            buzzerState = !buzzerState;   // Toggle buzzer state

            // Toggle Red LED and buzzer together
            digitalWrite(redLEDPin, buzzerState ? HIGH : LOW);

            if (buzzerState) {
                tone(buzzerPin, 1000);    // Sound Buzzer at 1kHz
            } else {
                noTone(buzzerPin);        // Turn off Buzzer
            }
        }
    } else if (distance > 4 && distance <=7) {
        // If object is closer than 2-5 cm
        digitalWrite(greenLEDPin, LOW);   // Turn off Green LED

        if (currentMillis - previousMillis >= buzzerInterval / 2) {
            // Sound and blink twice as frequently
            previousMillis = currentMillis;
            buzzerState = !buzzerState;   // Toggle buzzer state

            // Toggle Red LED and buzzer together
            digitalWrite(redLEDPin, buzzerState ? HIGH : LOW);

            if (buzzerState) {
                tone(buzzerPin, 2000);    // Sound Buzzer at 2kHz
            } else {
                noTone(buzzerPin);        // Turn off Buzzer
            }
        }
    } else if (distance <= 4) {
        // If object is closer than 2 cm
        digitalWrite(greenLEDPin, LOW);  // Turn off Green LED
        digitalWrite(redLEDPin, HIGH);   // Keep Red LED ON
        tone(buzzerPin, 3000);           // Continuous Buzzer at 3kHz
    } else {
        // If object is out of range (> 50 cm)
        digitalWrite(greenLEDPin, LOW);  // Turn off Green LED
        digitalWrite(redLEDPin, LOW);    // Turn off Red LED
        noTone(buzzerPin);               // Ensure buzzer is off
        buzzerState = false;             // Reset buzzer state
    }
}



void loop() {
    // Handle Bluetooth commands first
    if (Bluetooth.available()) {
        command = Bluetooth.read();
        Serial.print("Command: ");
        Serial.println(command);

        switch (command) {
            case 'F': case 'f': goForward(); break;  // Forward
            case 'B': case 'b': goBackward(); break; // Backward
            case 'L': case 'l': turnLeft(); break;   // Turn left
            case 'R': case 'r': turnRight(); break;  // Turn right
            case 'S': case 's': stopMotors(); break; // Stop
            default:
                Serial.println("Invalid command.");
                break;
        }
    }

    // Run parking sensor logic
    parkingSensor();
}
