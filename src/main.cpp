// #define HTX_TEST
#include "network.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU9250_asukiaaa.h>
#include <ArduinoJson.h>


//const int POWER_STEER_LIFT_PIN = 13;
const int servosEnablePin = 32;
const int liftControlServoPin = 12; //blue
const int steerControlServoPin = 14; //purpe
const int directionPin = 27; //high is forward, low is backward
const int manualPowerControlPin = 25;
const int hubPin = 13;
const int imuSDA = 21;
const int imuSCL = 22;
const int forcedDirectionPullUp = 26;
uint8_t powerSteerStatus;
uint8_t powerSteerStatusNet;
uint8_t powerSteerStatusNetOld;
uint8_t powerSteerStatusManual;
uint8_t powerSteerStatusManualOld;

char* MQTT_SUB_TOPIC = "actuator/assist";
const char* POWER_ASS_COMMAND = "powerAssCommand";
const char* POWER_CMD_ON = "on";
const char* POWER_CMD_OFF = "off";

DynamicJsonDocument doc(1024);

const uint8_t POWER_STEER_DISABLED = 0x00;
const uint8_t POWER_STEER_ENABLED = 0x01;

Servo myServo;  // create servo object to control a servo
Servo mySteering;
// 16 servo objects can be created on the ESP32
MPU9250_asukiaaa mySensor;

float gyroZThreshold = 200;
float accelXThreshold = 0.11;
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
void onReceiveCommand(char* topic, byte* payload, unsigned int length);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(directionPin, OUTPUT);
  pinMode(hubPin, OUTPUT);
  pinMode(manualPowerControlPin, INPUT);
  pinMode(servosEnablePin, OUTPUT);
  pinMode(forcedDirectionPullUp, OUTPUT); // temp
  digitalWrite(forcedDirectionPullUp, HIGH); // temp
  digitalWrite(directionPin, LOW);
  Wire.begin(imuSDA, imuSCL); //sda, scl/
#ifndef HTX_TEST
  net::connectToWifi();
  net::subscribeToMQTT(MQTT_SUB_TOPIC, onReceiveCommand);
  net::connectToMQTT();
#ifdef _ESP32_HAL_I2C_H_
  // for esp32
  // Serial.println("pre imu sda scl");
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
  xTaskCreate(&powerAssistUpdate, "powerassist", 10000, NULL, 0, NULL);
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
    // Serial.println("gyroZ: " + String(mySensor.gyroZ()));

    if (abs(mySensor.gyroZ()) > gyroZThreshold) {
      Serial.println("turning: servo move to 00");
      if (pos > 0) {
        for (int _pos = pos; _pos > 0; _pos -= 1) { // goes from 0 degrees to 180 degrees
          pos -= 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
      if (mySensor.gyroZ() > 0) { 
        // digitalWrite(directionPin, LOW); 
      }
      else {
        // digitalWrite(directionPin, HIGH); 
      }
      digitalWrite(hubPin, HIGH);
    }
    else {
      //Serial.println("no gyroZ");
      vTaskDelay(15);
      }
    

    //accelY hub motor
    // Serial.println("accelX: " + String(mySensor.accelX()));
    if (abs(mySensor.accelX()) > accelXThreshold) {
      Serial.println("forward/back: servo move to 90");
      if (pos < 90) {
        for (int _pos = pos; _pos < 180; _pos += 1) { // goes from 0 degrees to 180 degrees
          pos += 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
      if (mySensor.accelY() > 0) { 
        // digitalWrite(directionPin, HIGH);
      }
      else { 
        // digitalWrite(directionPin, LOW); 
        }
      digitalWrite(hubPin, HIGH);
    }
    else {
      //Serial.println("no accelY");
      vTaskDelay(20);
      }

    //accelX hub motor
    // Serial.println("accelY: " + String(mySensor.accelY()));
    if (abs(mySensor.accelY()) > accelYThreshold) {
      Serial.println("strafing: servo move to 00");
      if (pos > 0) {
        for (int _pos = pos; _pos > 0; _pos -= 1) { // goes from 0 degrees to 180 degrees
          pos -= 1; // in steps of 1 degree
          mySteering.write(_pos);    // tell servo to go to position in variable 'pos'
          vTaskDelay(MOTOR_DELAY_MS);
        }
      }
        if (mySensor.accelY() > 0) { 
          // digitalWrite(directionPin, LOW);
           }
        else {
          //  digitalWrite(directionPin, HIGH);
        }
        digitalWrite(hubPin, HIGH);
        vTaskDelay(MOTOR_DELAY_MS);
    }
    else {
      //Serial.println("no accelX");
      vTaskDelay(20);
      }

    if (abs(mySensor.gyroZ()) < gyroZThreshold && abs(mySensor.accelY()) < accelYThreshold && abs(mySensor.accelX()) < accelXThreshold) {
      digitalWrite(hubPin, LOW);
    }
    else vTaskDelay(20);
  }
}


void onReceiveCommand(char* topic, byte* payload, unsigned int length) {
  Serial.println("onreceie");
  char* buffer = (char*) payload;
  deserializeJson(doc, buffer);
  const char* cmd = doc[POWER_ASS_COMMAND];
  Serial.println(cmd);
  if (strcmp(cmd, POWER_CMD_ON) == 0) {
    powerSteerStatus = POWER_STEER_ENABLED;
    Serial.println("power on");
  } else if (strcmp(cmd, POWER_CMD_OFF) == 0) {
    powerSteerStatus = POWER_STEER_DISABLED;
    Serial.println("power off");
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
  // Serial.println("Motor Lowered");
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
  // Serial.println("Motor lifted");
}

void powerAssistUpdate(void* param) {
  for(;;) {
    net::checkConnection();
    // powerSteerStatusManual = digitalRead(manualPowerControlPin);
    // if (powerSteerStatusManual != powerSteerStatusManualOld) {
    //   powerSteerStatus = powerSteerStatusManual;
    //   powerSteerStatusManualOld = powerSteerStatusManual;
    //   powerSteerStatusNetOld = powerSteerStatusNet;
    // } else if (powerSteerStatusNet != powerSteerStatusNetOld) {
    //   powerSteerStatus = powerSteerStatusNet;
    //   powerSteerStatusManualOld = powerSteerStatusManual;
    //   powerSteerStatusNetOld = powerSteerStatusNet;
    // } else {
    //   vTaskDelay(20);
    //   continue;
    // }
    // Serial.printf("Power Steer status %d\n", powerSteerStatus);
#ifndef HTX_TEST
    switch(powerSteerStatus) {
      case POWER_STEER_ENABLED:
        // digitalWrite(servosEnablePin, HIGH);
        // digitalWrite(hubPin, LOW);
        lowerMotor();
        break;
      case POWER_STEER_DISABLED:
      default:
        liftMotor();
        // digitalWrite(servosEnablePin, LOW);
        // digitalWrite(hubPin, HIGH);
    }
#endif
    vTaskDelay(50);
  }
}