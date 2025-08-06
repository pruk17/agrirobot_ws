#include <Arduino.h>
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON parsing

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ready for JSON");
}

void loop() {
  if (Serial.available()) {
    String jsonString = Serial.readStringUntil('\n');
    jsonString.trim();

    JsonDocument doc;  
    DeserializationError error = deserializeJson(doc, jsonString);


    if (!error) {
      float x = doc["x"];
      float y = doc["y"];
      bool press = doc["press"];

      Serial.print("x: "); Serial.print(x, 3);
      Serial.print(" y: "); Serial.print(y, 3);
      Serial.print(" press: "); Serial.println(press ? "YES" : "NO");
        // Here you can add logic to control a servo or motor based on the received values
    } else {
      Serial.print("JSON Error: ");
      Serial.println(error.c_str());
    }
  }
}
