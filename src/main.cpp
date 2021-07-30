#include "RpiFunctions.hpp"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU9250_asukiaaa.h>

//const int POWER_STEER_LIFT_PIN = 13;
const int liftServoPin = 18;
const int steerServoPin = 12;
const int directionPin = 19; //high is forward, low is backward
const int hubPin = 13;
uint8_t powerSteerStatus;

const uint8_t POWER_STEER_ENABLED = 0x01;
const uint8_t POWER_STEER_DISABLED = 0x02;

#define SDA 21
#define SCL 22
#define imu_SDA_PIN 23
#define imu_SCL_PIN 5


const int16_t i2c_esp32=0x06;


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
  // Rpi::initWireSlave(SDA, SCL, i2c_esp32);
  // Rpi::registerReceiveRequest(onReceiveCommand);

  #ifdef _ESP32_HAL_I2C_H_
  // for esp32
  Wire.begin(imu_SDA_PIN, imu_SCL_PIN); //sda, scl
  #else
  Wire.begin();
  #endif

  initServo();
  initIMU();

  mySteering.write(90);
  myServo.write(0);

  // xTaskCreate Definition: https://www.freertos.org/a00125.html
  xTaskCreate(&IMUUpdate, "IMU", 10000, NULL, 1, NULL);
  // xTaskCreate(&powerAssistUpdate, "powerassist", 10000, NULL, 0, NULL);
  
  
}


void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(POWER_STEER_LIFT_PIN, HIGH);
  //delay(100);
  mySensor.gyroUpdate();
  mySensor.accelUpdate();
}

void initServo() {
  Serial.println("starting servos");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);    // standard 50 hz servo
  myServo.attach(liftServoPin, minServoFreq, maxServoFreq); // attaches the servo on pin 18 to the servo object
  mySteering.setPeriodHertz(50);    // standard 50 hz servo
  mySteering.attach(steerServoPin, minServoFreq, maxServoFreq); // attaches the servo on pin 18 to the servo object
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
    //Serial.println("accelY: " + String(mySensor.accelY()));
    if (abs(mySensor.accelY()) > accelYThreshold) {
      Serial.println("forward/back: servo move to 90");
      if (pos < 90) {
        for (int _pos = pos; _pos < 90; _pos += 1) { // goes from 0 degrees to 180 degrees
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

void powerAssistUpdate(void* param) {
  for(;;) {
    Rpi::updateWire(); // Invokes callback functions if data is availabl
    switch(powerSteerStatus) {
      case POWER_STEER_ENABLED:
        for (int i = 0; i < 90; i++) {
          myServo.write(i);
        }
        break;
      case POWER_STEER_DISABLED:
      default:
        for (int i = 90; i > 0; i++) {
          myServo.write(i);
        }
    }
    taskYIELD();
  }
}

// /* Sweep
//  by BARRAGAN <http://barraganstudio.com>
//  This example code is in the public domain.

//  modified 8 Nov 2013
//  by Scott Fitzgerald

//  modified for the ESP32 on March 2017
//  by John Bennett

//  see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

//  * Different servos require different pulse widths to vary servo angle, but the range is 
//  * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
//  * sweep 180 degrees, so the lowest number in the published range for a particular servo
//  * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
//  * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
//  * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
//  * degrees.
//  * 
//  * Circuit: (using an ESP32 Thing from Sparkfun)
//  * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
//  * the ground wire is typically black or brown, and the signal wire is typically yellow,
//  * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
//  * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
//  * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
//  * 
//  * We could also connect servo power to a separate external
//  * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
//  * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
//  * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
//  * 
//  * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
//  * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
//  * These values actually drive the servos a little past 0 and 180, so
//  * if you are particular, adjust the min and max values to match your needs.
//  */

// #include <ESP32Servo.h>
// #include <MPU9250_asukiaaa.h>
 
// #ifdef _ESP32_HAL_I2C_H_
// #define SDA_PIN 23
// #define SCL_PIN 5
// #endif

// Servo myServo;  // create servo object to control a servo
// //Servo mySteering;
// // 16 servo objects can be created on the ESP32
// MPU9250_asukiaaa mySensor;

// float gyroZThreshold = 200;
// float accelXThreshold = 0.8;
// float accelYThreshold = 0.8;

// int pos = 0;    // variable to store the servo position
// // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// int servoPin = 18;

// void setup() {
//   // Allow allocation of all timers
//   ESP32PWM::allocateTimer(0);
//   ESP32PWM::allocateTimer(1);
//   ESP32PWM::allocateTimer(2);
//   ESP32PWM::allocateTimer(3);
//   myServo.setPeriodHertz(50);    // standard 50 hz servo
//   myServo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
//   // using default min/max of 1000us and 2000us
//   // different servos may require different min/max settings
//   // for an accurate 0 to 180 sweep

//  Serial.begin(115200);
// Serial.println("started");
 
// #ifdef _ESP32_HAL_I2C_H_
// // for esp32
// Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
// #else
// Wire.begin();
// #endif
 
// //mySensor.setWire(&Wire);
 
// mySensor.beginAccel();
// mySensor.beginGyro();

// myServo.write(90);
// }

// void loop() {
// //mySensor.accelUpdate();
// //Serial.println("accelX: " + String(mySensor.accelX()));
// //Serial.println("accelY: " + String(mySensor.accelY()));

// mySensor.gyroUpdate();
// Serial.println("gyroZ: " + String(mySensor.gyroZ()));


//  if (mySensor.gyroZ() > gyroZThreshold && pos < 90) {
//   Serial.println("servo move to 90");
//   Serial.println("servo move to 90");
//   Serial.println("servo move to 90");
//   for (int _pos = pos; _pos <= 90; _pos += 1) { // goes from 0 degrees to 180 degrees
//     pos += 1; // in steps of 1 degree
//     myServo.write(_pos);    // tell servo to go to position in variable 'pos'
//     delay(15);             // waits 15ms for the servo to reach the position
//   }
//   }
// else if (mySensor.gyroZ() < -gyroZThreshold && pos > 0) {
//   Serial.println("servo move to 00");
//   Serial.println("servo move to 00");
//   Serial.println("servo move to 00");
//   for (int _pos = pos; _pos >= 0; _pos -= 1) { // goes from 180 degrees to 0 degrees
//     pos -= 1;
//     myServo.write(_pos);    // tell servo to go to position in variable 'pos'
//     delay(15);             // waits 15ms for the servo to reach the position
//   }
//  }
// }