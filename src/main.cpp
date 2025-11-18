#include <Arduino.h>

// User specified GPIO23 for LED
#define LED_PIN 23

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  Serial.println("Blink");
}
