/*
 * Main file
 * 
 * apply a duty cycle of 255/ref to the motor1 (PWM) where ref is a value obtained from the serial port (in ascii format) 
 * send back the actual position of the motor shaft (in pulses).
 * 
 */


#include "Motors.h"
#include "Encoders.h"
#include "PID.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include "MeRGBLineFollower.h"
#include "MeUltrasonicSensor.h"
//#include "MeMegaPi.h"

MeUltrasonicSensor ultraSensor(PORT_8);
MeCompass MeCompass(PORT_6, ADDRESS3);
MeRGBLineFollower LineFollower(PORT_7, ADDRESS1);
MeGyro gyro;
int state = 0;
float ang;
#define DT 50  //sampling period in milliseconds
float offset;
#define speed_limit 280
int getDelaySensorWheel(float tension);
int delayt = getDelaySensorWheel(12*190/255);
float kp;
int ref = 120;
bool flagCurveLock = true;
float lastLock = millis();
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
  if (millis()-lastLock > 600){
    flagCurveLock = true;
  }
  LineFollower.loop();
  int offset = LineFollower.getPositionOffset();
  gyro.update();
  
  static int ref = 190;  // reference signal
  ang = gyro.getAngleZ();
  int u = ref;  // control signal

  //waitNextPeriod();

  Serial.println(offset);
  Serial.println(state);
  Serial.println(ang);

  // get the new reference from the serial port is any.
  if (Serial.available() > 1) {  // something to read ?
    ref = Serial.parseInt();     // parse the value (ascii -> int)
  }
/*
  if (ang <= 45 && ang >= -45){
    state = 0;
  }
  else if (ang <= -45 && ang >= -135){
    state = 2;
  }
  else if (ang >= 45 && ang <= 135){
    state = 1;
  }
  else if (ang >= 135 || ang <= -135){
    state = 3;
  }
*/
  if (state == 0){
    if ((ang >= 80 ) && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 1;
      lastLock = millis();
      flagCurveLock = false;
    }
    else if ((ang <= -80 ) && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 2;
      lastLock = millis();
      flagCurveLock = false;
    }

    else if ((offset>=10 || offset <= -10) && flagCurveLock){

    u = 45;
    kp = 0.08;
    LineFollower.setKp(kp);
    pid_control_curve(u,offset);
    
  }

    else {
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
    }
    
  } 

  else if (state == 1){
    if (ang <=10 && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 0;
      lastLock = millis();
      flagCurveLock = false;
    }
    else if (ang >=170 && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 3;
      lastLock = millis();
      flagCurveLock = false;
    }
    else if ((offset>=10 || offset <= -10) && flagCurveLock){

    u = 45;
    kp = 0.08;
    LineFollower.setKp(kp);
    pid_control_curve(u,offset);
    
  }

    else {
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
    }

  }

  else if (state == 2){
    if (ang <=-170 && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 3;
      lastLock = millis();
      flagCurveLock = false;
    }
    else if (ang >=-10  && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 0;
      lastLock = millis();
      flagCurveLock = false;
    }

    else if ((offset>=10 || offset <= -10) && flagCurveLock){

    u = 45;
    kp = 0.08;
    LineFollower.setKp(kp);
    pid_control_curve(u,offset);
    
  }

    else {
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
    }
  }

  else if (state == 3){
    if ((ang >=-100) && (ang < 0) && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 2;
      lastLock = millis();
      flagCurveLock = false;

    }
    else if ((ang <=100) && (ang > 0) && (offset >=-30 || offset <= 30)){
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
      state = 1;
      lastLock = millis();
      flagCurveLock = false;
    }

    else if ((offset>=10 || offset <= -10) && flagCurveLock){

    u = 45;
    kp = 0.08;
    LineFollower.setKp(kp);
    pid_control_curve(u,offset);
    
  }

    else {
      u = 150;
      kp = 0.04;
      LineFollower.setKp(kp);
      pid_control(u,offset);
    }
  }

  
  /*if (offset >= 23 || offset <= -23){
    delay(delayt*10);
    
    if (offset>0){
        setMotorAVoltage(-(0.25*u*0.942));
        
    }
    else{
        setMotorBVoltage(-(0.25*u));
    }
    delay(320);
  }
  */

  //setMotorBVoltage(u);
  //setMotorAVoltage(0.942*u);
  
}
//COMENTEI AQUI

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

void pid_control_curve(int motor_speed, int sensor_value) {
  //setMotorAVoltage(0);
  //setMotorBVoltage(0);
  int error = sensor_value; // between -512 and 512
  proportional = p_gain_curve * (error)*motor_speed/30; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*motor_speed/35;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit && motor_speed > 0) {
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
