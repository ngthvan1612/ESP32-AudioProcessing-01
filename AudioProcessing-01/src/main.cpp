#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

int counter = 0;

void loop() {
  Serial.println("OK");
  delay(1000);
}