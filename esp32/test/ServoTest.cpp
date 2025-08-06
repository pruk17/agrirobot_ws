#include <Arduino.h>
#include <ESP32Servo.h> // Include the Servo library for servo control

const int SERVO_PIN = 15; // Define the GPIO pin for the servo
Servo servo; // Create a Servo object

void setup() {
  Serial.begin(115200);
  Serial.println("Servo Test Started");
  servo.attach(SERVO_PIN); // Attach the servo to the defined pin
}

void loop() {
  // This is a placeholder for servo test logic
  if (Serial.available()) {
    char keyboard = Serial.read();
    if (keyboard == '\n' || keyboard == '\r') return;
    switch (keyboard){
    case ('a'):
        servo.write(0); // Move servo to position 0
        Serial.println("Servo moved to position 0");
        delay(100); 
        break;
    case ('s'):
        servo.write(90); // Move servo to position 90
        Serial.println("Servo moved to position 90");
        delay(100);
        break;
    case ('d'):
        servo.write(180); // Move servo to position 180
        Serial.println("Servo moved to position 180");
        delay(100);
        break;    
    default:
        break;
    }
    /*
    if (keyboard == 'a') {
      // Simulate moving servo to position 1
      Serial.println("Moving servo to position 1");
      digitalWrite(SERVO_PIN, HIGH); // Simulate servo action
      delay(1000); // Wait for a second
      digitalWrite(SERVO_PIN, LOW); // Stop servo action
    } else if (keyboard == 's') {
      // Simulate stopping the servo
      Serial.println("Stopping servo");
      digitalWrite(SERVO_PIN, LOW);
    } else {
      Serial.println("Unknown command");
    }*/
  }
}