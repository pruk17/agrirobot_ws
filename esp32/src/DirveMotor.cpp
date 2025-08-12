#include <Arduino.h>
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON parsing
#include "esp_sleep.h"// Include ESP32 sleep library for deep sleep functionality
#include "driver/uart.h"

unsigned long last_activity_time = 0;
const unsigned long inactivity_timeout = 5000; // 5 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup cause: ");
  Serial.println(cause);

  // Enable UART wakeup and timer wakeup
  esp_sleep_enable_uart_wakeup(UART_NUM_0);
  esp_sleep_enable_timer_wakeup(inactivity_timeout * 1000); // in microseconds

  last_activity_time = millis();

  Serial.println("Ready to receive commands.");
}

void loop() {
  if (Serial.available()) {
    String jsonString = Serial.readStringUntil('\n'); // read 1 line of JSON data
    jsonString.trim(); // delete space, \r\n 
    Serial.print("Received: ");
    Serial.println(jsonString);
    StaticJsonDocument<200> doc; // Create a JSON document with a capacity of 200 bytes
                                // Parse the JSON string into the document
    DeserializationError error = deserializeJson(doc, jsonString);
    last_activity_time = millis(); // Update the last activity time

    if (!error) {
      // detect the key "cmd" to use Keyboard mode
      if (doc.containsKey("cmd")) {
        const char* cmd = doc["cmd"];
        Serial.print("Keyboard command: ");
        Serial.println(cmd);

        if (strcmp(cmd, "forward") == 0) {
          Serial.println("Motor: FORWARD");
        }
        else if (strcmp(cmd, "backward") == 0) {
          Serial.println("Motor: BACKWARD");
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
        }
        else if (y < -0.5) {
          Serial.println("Motor: BACKWARD");
        }
      }
    } 

  } 
  else {
    // No data available, check inactivity
    if (millis() - last_activity_time > inactivity_timeout) {
      Serial.println("No activity for 5 seconds, entering light sleep...");
      esp_light_sleep_start();
      Serial.println("Woke up from light sleep.");
      last_activity_time = millis(); // reset timer after wakeup
    }
  }

  delay(10);
  // Check if the device should enter deep sleep
}
