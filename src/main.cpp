// #define HTX_TEST
// #define NO_RPI_TEST
#ifndef NO_RPI_TEST
#include "RpiFunctions.hpp"
#endif
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU9250_asukiaaa.h>


//const int POWER_STEER_LIFT_PIN = 13;
const int servosEnablePin = 14;
const int liftControlServoPin = 18;
const int steerControlServoPin = 12;
const int directionPin = 19; //high is forward, low is backward
const int manualPowerControlPin = 17;
const int hubPin = 13;
uint8_t powerSteerStatus;

const uint8_t POWER_STEER_DISABLED = 0x00;
const uint8_t POWER_STEER_ENABLED = 0x01;

// #define SDA 21
// #define SCL 22
#define imu_SDA_PIN 21
#define imu_SCL_PIN 22

const int16_t i2c_esp32=0x05;

Servo myServo;  // create servo object to control a servo
Servo mySteering;
// 16 servo objects can be created on the ESP32
MPU9250_asukiaaa mySensor;

float gyroZThreshold = 200;
float accelXThreshold = 0.8;
float accelYThreshold = 0.8;
float MOTOR_DELAY_MS = 15;
int minServoFreq = 500;
int maxServoFreq = 2400;

int pos = 90;    // variable to store the servo position
int liftServoPos = 0;
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 

void initServo();

void initIMU();

// A periodic task. Reads IMU, then handles turning left/right/reverse
void IMUUpdate(void *param);

// A periodic task. 
void powerAssistUpdate(void* param);

// Callback from receiving data from RPI, Should handle the command to turn on/off power steering
void onReceiveCommand(uint8_t registerCode, int howMany, uint8_t* data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(directionPin, OUTPUT);
  pinMode(hubPin, OUTPUT);
  pinMode(manualPowerControlPin, INPUT);
  pinMode(servosEnablePin, OUTPUT);
  // Serial.println("Pre rpi init");
#ifndef NO_RPI_TEST
  // Rpi::initWireSlave(SDA, SCL, i2c_esp32);
  // Rpi::registerReceiveRequest(onReceiveCommand);
#endif

#ifndef HTX_TEST
#ifdef _ESP32_HAL_I2C_H_
  // for esp32
  // Serial.println("pre imu sda scl");
  Wire.begin(imu_SDA_PIN, imu_SCL_PIN); //sda, scl/
#else
  Wire.begin();
#endif
  // Serial.println("preservoimu");
  initServo();
  // Serial.println("servo init");
  initIMU();
  Serial.println("Servero and imu init");
  delay(5000);
  mySteering.write(0);
  myServo.write(0);
  xTaskCreate(&IMUUpdate, "IMU", 10000, NULL, 1, NULL);
#endif // ifdef HTX_TEST
  // xTaskCreate(&powerAssistUpdate, "powerassist", 10000, NULL, 0, NULL);
}


void loop() {
#ifndef HTX_TEST
  //digitalWrite(POWER_STEER_LIFT_PIN, HIGH);
  //delay(100);
  mySensor.gyroUpdate();
  mySensor.accelUpdate();
  // Rpi::updateWire();
#endif // ifndef HTX_TEST
}

void initServo() {
  Serial.println("starting servos");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);    // standard 50 hz servo
  myServo.attach(liftControlServoPin, minServoFreq, maxServoFreq); // attaches the servo on pin 18 to the servo object
  mySteering.setPeriodHertz(50);    // standard 50 hz servo
  mySteering.attach(steerControlServoPin, minServoFreq, maxServoFreq); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void initIMU() {
  Serial.println("starting IMU");  
  mySensor.beginAccel();
  mySensor.beginGyro();
}

void IMUUpdate(void *param) {
  for (;;) {
    //Serial.println("pos: " + String(pos));
    //gyro steering
    //Serial.println("gyroZ: " + String(mySensor.gyroZ()));

    if (abs(mySensor.gyroZ()) > gyroZThreshold) {
      Serial.println("turning: servo move to 00");
      if (pos > 0) {
        for (int _pos = pos; _pos > 0; _pos -= 1) { // goes from 0 degrees to 180 degrees
          pos -= 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
      if (mySensor.gyroZ() > 0) digitalWrite(directionPin, LOW); 
      else digitalWrite(directionPin, HIGH);
      digitalWrite(hubPin, HIGH);
    }
    else {
      //Serial.println("no gyroZ");
      vTaskDelay(15);
      }
    

    //accelY hub motor
    Serial.println("accelY: " + String(mySensor.accelY()));
    if (abs(mySensor.accelY()) > accelYThreshold) {
      Serial.println("forward/back: servo move to 90");
      if (pos < 90) {
        for (int _pos = pos; _pos < 180; _pos += 1) { // goes from 0 degrees to 180 degrees
          pos += 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
      if (mySensor.accelY() > 0) digitalWrite(directionPin, HIGH); 
      else digitalWrite(directionPin, LOW);
      digitalWrite(hubPin, HIGH);
    }
    else {
      //Serial.println("no accelY");
      taskYIELD();
      }

    //accelX hub motor
    //Serial.println("accelX: " + String(mySensor.accelX()));
    if (abs(mySensor.accelX()) > accelXThreshold) {
      Serial.println("strafing: servo move to 00");
      if (pos > 0) {
        for (int _pos = pos; _pos > 0; _pos -= 1) { // goes from 0 degrees to 180 degrees
          pos -= 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
        if (mySensor.accelX() > 0) digitalWrite(directionPin, LOW); 
        else digitalWrite(directionPin, HIGH);
        digitalWrite(hubPin, HIGH);
        vTaskDelay(MOTOR_DELAY_MS);
    }
    else {
      //Serial.println("no accelX");
      taskYIELD();
      }

    if (abs(mySensor.gyroZ()) < gyroZThreshold && abs(mySensor.accelY()) < accelYThreshold && abs(mySensor.accelX()) < accelXThreshold) {
      digitalWrite(hubPin, LOW);
    }
    else taskYIELD();
  }
}


void onReceiveCommand(uint8_t registerCode, int howMany, uint8_t* data) {
  switch(registerCode) {
    case 0x01:
      powerSteerStatus = data[0];
      break;
  }
}

void lowerMotor() {
  Serial.println("Lowering motor");
  for (; liftServoPos < 90; liftServoPos++) {
    if (powerSteerStatus == POWER_STEER_DISABLED) {
      return;
    } 
    myServo.write(liftServoPos);
    vTaskDelay(MOTOR_DELAY_MS);
  }
  Serial.println("Motor Lowered");
}

void liftMotor() {
  Serial.println("Lifting motor");
  for (; liftServoPos > 0; liftServoPos--) {
    if (powerSteerStatus == POWER_STEER_ENABLED) {
      return;
    }
    myServo.write(liftServoPos);
    vTaskDelay(MOTOR_DELAY_MS);
  }
  Serial.println("Motor lifted");
}

void powerAssistUpdate(void* param) {
  for(;;) {
#ifndef NO_RPI_TEST
    Rpi::updateWire(); // Invokes callback functions if data is available
#else
    powerSteerStatus = digitalRead(manualPowerControlPin);
#endif
    Serial.printf("Power Steer status %d\n", powerSteerStatus);
#ifndef HTX_TEST
    switch(powerSteerStatus) {
      case POWER_STEER_ENABLED:
        digitalWrite(servosEnablePin, HIGH);
        lowerMotor();
        break;
      case POWER_STEER_DISABLED:
      default:
        liftMotor();
        digitalWrite(servosEnablePin, LOW);
    }
#endif
    vTaskDelay(50);
  }
}