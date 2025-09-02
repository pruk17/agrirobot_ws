#include <Arduino.h>
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON parsing

#define M1A 16
#define M2A 17

#define M1B 18
#define M2B 19

void brake() {
  digitalWrite(M1A, LOW); digitalWrite(M2A, LOW);
  digitalWrite(M1B, LOW); digitalWrite(M2B, LOW);
}

void forward() {
  digitalWrite(M1A, HIGH); digitalWrite(M2A, HIGH);
  digitalWrite(M1B, LOW); digitalWrite(M2B, LOW);
}

void backward() {
  digitalWrite(M1A, LOW); digitalWrite(M2A, LOW);
  digitalWrite(M1B, HIGH); digitalWrite(M2B, HIGH);
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Ready to receive commands.");

  // Motor pins as output
  pinMode(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2B, OUTPUT);

  // Ensure motors are stopped at startup
  brake();
}

void loop() {
  if (Serial.available()) {
    String jsonString = Serial.readStringUntil('\n'); // read 1 line of JSON data
    jsonString.trim(); // delete space, \r\n 
    Serial.print("Received: ");
    Serial.println(jsonString);
    JsonDocument doc; // Create a JSON document (dynamic)
                                // Parse the JSON string into the document
    DeserializationError error = deserializeJson(doc, jsonString);

    if (!error) {
      // detect the key "cmd" to use Keyboard mode
      if (doc["cmd"].is<const char*>()) {
        const char* cmd = doc["cmd"];
        Serial.print("Keyboard command: ");
        Serial.println(cmd);

        if (strcmp(cmd, "forward") == 0) {
          Serial.println("Motor: FORWARD");
          forward();
        }
        else if (strcmp(cmd, "backward") == 0) {
          Serial.println("Motor: BACKWARD");
          backward();
        }
        else if (strcmp(cmd, "stop") == 0) { 
          // Stop motor when "E" is pressed
          Serial.println("Motor: STOP (brake)");
          brake();
        }
      }
      // No "cmd" use Joystick
      else {
        float x = doc["x"];
        float y = doc["y"];
        bool press = doc["press"];

        Serial.print("x: "); Serial.print(x, 3);
        Serial.print(" y: "); Serial.print(y, 3);
        // =====  joystick =====
        //  y > 0.5 = forward, y < -0.5 = backward
        if (y > 0.5) {
          Serial.println("Motor: FORWARD");
          forward();
        }
        else if (y < -0.5) {
          Serial.println("Motor: BACKWARD");
          backward();
        }
        else {
          Serial.println("Motor: STOP");
          brake();
        }
      }
    } 

  } 
  delay(10);
  // Check if the device should enter deep sleep
}
