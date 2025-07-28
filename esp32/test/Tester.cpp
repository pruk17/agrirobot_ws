#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Ready");
}

void loop() {
  // Send simple text message every second
  Serial.println("waiting...");
  delay(1000);
}
