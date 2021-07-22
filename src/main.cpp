#include <Arduino.h>

const int POWER_STEER_PIN = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(POWER_STEER_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(POWER_STEER_PIN, LOW);
  delay(5000);
  digitalWrite(POWER_STEER_PIN, HIGH);
  delay(5000);
}