#include <Arduino.h>

// Left motor (Motor A)
#define IN1 14
#define IN2 13

// Right motor (Motor B)
#define IN3 26
#define IN4 27

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  Serial.println("ESP32 Ready to receive motor commands");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w':
        forward();
        Serial.println("Forward");
        break;
      case 's':
        backward();
        Serial.println("Backward");
        break;
      case 'x':
        stopMotors();
        Serial.println("Stop");
        break;
      default:
        Serial.println("Invalid command");
        break;
    }
  }
}
// This code controls two DC motors using an ESP32.