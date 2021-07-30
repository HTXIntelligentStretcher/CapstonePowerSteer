#include "RpiFunctions.hpp"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU9250_asukiaaa.h>

const int POWER_STEER_PIN = 13;
uint8_t powerSteerStatus;

const uint8_t POWER_STEER_ENABLED = 0x01;
const uint8_t POWER_STEER_DISABLED = 0x02;

#define SDA 21
#define SCL 22

const int16_t i2c_esp32=0x06;


Servo myServo;  // create servo object to control a servo
//Servo mySteering;
// 16 servo objects can be created on the ESP32
MPU9250_asukiaaa mySensor;

float gyroZThreshold = 200;
float accelXThreshold = 0.8;
float accelYThreshold = 0.8;
float MOTOR_DELAY_MS = 15;

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 18;

void onReceiveCommand(uint8_t registerCode, int howMany, uint8_t* data);
void IMUUpdate(void *param);
void powerAssistUpdate(void* param);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(POWER_STEER_PIN, OUTPUT);
  Rpi::initWireSlave(SDA, SCL, i2c_esp32);
  Rpi::registerReceiveRequest(onReceiveCommand);
  xTaskCreate(&IMUUpdate, "IMU", 10000, NULL, 1, NULL);
  xTaskCreate(&powerAssistUpdate, "powerassist", 10000, NULL, 0, NULL);
}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(POWER_STEER_PIN, HIGH);
  delay(100);

}

void initServo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);    // standard 50 hz servo
  myServo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void initIMU() {
    
  mySensor.beginAccel();
  mySensor.beginGyro();

}

void IMUUpdate(void *param) {
  for (;;) {
    mySensor.gyroUpdate();
    Serial.println("gyroZ: " + String(mySensor.gyroZ()));

    if (mySensor.gyroZ() > gyroZThreshold && pos < 90) {
      Serial.println("servo move to 90");
      Serial.println("servo move to 90");
      Serial.println("servo move to 90");
      for (_pos = pos; _pos <= 90; _pos += 1) { // goes from 0 degrees to 180 degrees
        pos += 1; // in steps of 1 degree
        myServo.write(_pos);    // tell servo to go to position in variable 'pos'
        vTaskDelay(MOTOR_DELAY_MS);
      }
    }
    else if (mySensor.gyroZ() < -gyroZThreshold && pos > 0) {
      Serial.println("servo move to 00");
      Serial.println("servo move to 00");
      Serial.println("servo move to 00");
      for (_pos = pos; _pos >= 0; _pos -= 1) { // goes from 180 degrees to 0 degrees
        pos -= 1;
        myServo.write(_pos);    // tell servo to go to position in variable 'pos'
        vTaskDelay(MOTOR_DELAY_MS);
      }
    }
  }
}


void onReceiveCommand(uint8_t registerCode, int howMany, uint8_t* data) {
  switch(registerCode) {
    case 0x01:
      powerSteerStatus = data[0];
      break;
  }
}

void powerAssistUpdate(void* param) {
  for(;;) {
    Rpi::updateWire();
    switch(powerSteerStatus) {
      case POWER_STEER_ENABLED:
        digitalWrite(POWER_STEER_PIN, HIGH);
        break;
      case POWER_STEER_DISABLED:
      default:
        digitalWrite(POWER_STEER_PIN, LOW);
    }
    taskYIELD();
  }
}