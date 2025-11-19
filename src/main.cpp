#include <Arduino.h>

// User specified GPIO23 for LED
#define LED_PIN 23

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  // Wait for serial connection for up to 5 seconds
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000);
  Serial.println("MnB Ultralight N17 Firmware Initialized");
}

void loop() {
  // Heartbeat
  static unsigned long last_blink = 0;
  if (millis() - last_blink > 500) {
    last_blink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Simple echo for testing
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Echo: ");
    Serial.println(c);
  }
}
