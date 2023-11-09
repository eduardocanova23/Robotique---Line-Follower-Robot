#include "Motors.h"
#include "Encoders.h"
#include "PID.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//#include <MeAuriga.h>
#include "MeRGBLineFollower.h"
#include "MeUltrasonicSensor.h"
#include "MeMegaPi.h"

MeUltrasonicSensor ultraSensor(PORT_8);
MeCompass MeCompass(PORT_6, ADDRESS3);
MeRGBLineFollower LineFollower(PORT_7, ADDRESS1);
MeGyro gyro;
int state = 0;
// state 0 straight line
// state 1 do a curve
// state 2 dodge the object
// state 3 wait the other car to pass
float ang;
float lastang;
#define DT 50  //sampling period in milliseconds
float offset;
#define speed_limit 280
int getDelaySensorWheel(float tension);
int delayt = getDelaySensorWheel(12*190/255);
float kp;
int ref = 120;
bool flagCurveLock = true;
float lastLock = millis();
bool detectedObstacle = false;
int uAd = -25;
int uBd = 25;
int uAe = -uAd;
int uBe = -uBd;

// BOTAR TODA ESSA TRALHA EM UMA FUNCAO PRA NAO FICAR FEIO

void setup() {
  

  gyro.begin();
  LineFollower.begin();
  kp = 0.05;
  LineFollower.setKp(kp);

  // initialization of the serial communication.
  Serial.begin(9600);
  Serial.setTimeout(10);


}

void loop() {
  // Main loop
  float dist = ultraSensor.distanceCm();
  
  if (millis()-lastLock > 700){
    flagCurveLock = true;
  }
  LineFollower.loop();
  int offset = LineFollower.getPositionOffset();
  gyro.update();
  
  static int ref = 190;  // reference signal
  ang = gyro.getAngleZ();
  int u = ref;  // control signal

  //waitNextPeriod();

  //Serial.println(offset);
  //Serial.println(state);
  //Serial.println(ang);

  // get the new reference from the serial port is any.
  if (Serial.available() > 1) {  // something to read ?
    ref = Serial.parseInt();     // parse the value (ascii -> int)
  }

  while (state == 0){
    LineFollower.setKp(0.04);
    LineFollower.loop();
    int offset = LineFollower.getPositionOffset();

    gyro.update();
    ang = gyro.getAngleZ();

    float dist = ultraSensor.distanceCm();

    pid_control(160,offset);

    if (millis()-lastLock > 600){
      // if some time passed since last lock, unlock the curve
      flagCurveLock = true;
    }

    if ((offset>=13 || offset <= -13) && flagCurveLock){
      state = 1; // starting to detect the curve -> change the state to curve
    }

    if (dist < 15){
      state = 2; // detected obstacle -> change the state to dodge
    }

  }

  lastang = gyro.getAngleZ();
  delay(70);

  float timebeforecurve = millis();
  while (state == 1 && flagCurveLock){
    LineFollower.loop();
    LineFollower.setKp(0.08);
    int offset = LineFollower.getPositionOffset();

    gyro.update();
    ang = gyro.getAngleZ();
    
    if (offset > 0){
      pid_control_curve(-25,25,offset);
    }

    else{
      pid_control_curve(25,-25,offset);
    }
    float dist = ultraSensor.distanceCm();
    


    if ((abs(abs(lastang) - abs(ang)) >= 82) && (offset >= -40 && offset <= 40)){ 
      // if the difference between the angle when the curve started and now is 90, then change state
      // and lock the curve for some time 
      state = 0;
      flagCurveLock = false;
      lastLock = millis();
    }

    if ((millis() - timebeforecurve) > 2000){
      state = 0;
      flagCurveLock = false;
      lastLock = millis();
    }

    if (dist < 15){
      state = 2; // detected obstacle -> change the state to dodge
    }
  }

  float timebeforedodge = millis();
  while (state == 2 &&  (millis() - timebeforedodge < 3000)){


    LineFollower.loop();
    int offset = LineFollower.getPositionOffset();

    gyro.update();
    ang = gyro.getAngleZ();
    
    float dist = ultraSensor.distanceCm();

    LineFollower.setKp(0.08);
    pid_control_dodge(25,offset);

  }
  //setMotorBVoltage(u);
  //setMotorAVoltage(0.942*u);
  
}

void pid_control(int motor_speed, int sensor_value) {
  int error = sensor_value; // between -512 and 512
  proportional = p_gain * (error)*motor_speed/40; // replace 10 by max value of steering fct
  integral += (i_gain * error)/1000;
  derivative = d_gain * (error - last_error)*motor_speed/75;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit && motor_speed > 0) {
    setMotorAVoltage((motor_speed - delta_speed)*0.942);
    setMotorBVoltage(motor_speed + delta_speed);
  }
  last_error = error;
  //Serial.println(delta_speed);
  
}

void pid_control_curve(int motor_speedA, int motor_speedB, int sensor_value) {
  //setMotorAVoltage(0);
  //setMotorBVoltage(0);
  int error = sensor_value; // between -512 and 512
  proportional = p_gain_curve * (error)*abs(motor_speedB)/30; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*abs(motor_speedB)/35;

  float delta_speed = (proportional + derivative + integral);
  if (abs(motor_speedB) + delta_speed < speed_limit) {
    setMotorAVoltage((motor_speedA - delta_speed)*0.942);
    setMotorBVoltage(motor_speedB + delta_speed);
  }
  last_error = error;
  if ((motor_speedA < 10 && motor_speedB < 10) || error == 0){
    state = 0;
    flagCurveLock = false;
    lastLock = millis();
  }
}

void pid_control_dodge(int motor_speed, int sensor_value) {
  //setMotorAVoltage(0);
  //setMotorBVoltage(0);
  int error = sensor_value; // between -512 and 512
  proportional = p_gain_curve * (error)*motor_speed/30; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*motor_speed/35;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit) {
    setMotorAVoltage((motor_speed - delta_speed)*0.942);
    setMotorBVoltage(motor_speed + delta_speed);
  }
  last_error = error;
  //Serial.println(delta_speed);
  //delay(100);
}

int getDelaySensorWheel(float tension) {
    float dist = 191.501;
    float speed = 4.737272E-1 + 3.324745E-1 * tension - 6.857634E-2 * pow(tension, 2) + 7.717948E-3 * pow(tension, 3) - 4.44638E-4 * pow(tension, 4) + 1.025641E-5 * pow(tension, 5);
    float time = dist / (1000.0 * speed);
    time = (int)time;
    //Serial.println(speed);
    return time;
}

void waitNextPeriod() {
  static long LastMillis = 0;
  long timeToWait = DT - (millis() - LastMillis);
  if (timeToWait > 0)
    delay(timeToWait);
  LastMillis = millis();
}
