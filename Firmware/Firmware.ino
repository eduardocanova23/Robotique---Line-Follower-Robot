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


MeUltrasonicSensor ultraSensor(PORT_8);
MeCompass MeCompass(PORT_6, ADDRESS3);
MeRGBLineFollower LineFollower(PORT_7, ADDRESS1);


#define DT 50  //sampling period in milliseconds

#define speed_limit 280
int getDelaySensorWheel(float tension);
int delayt = getDelaySensorWheel(12*190/255);

void setup() {
  
  
  LineFollower.begin();
  LineFollower.setKp(0.05);

  // initialization of the serial communication.
  Serial.begin(9600);
  Serial.setTimeout(10);


}

void loop() {
  // Main loop
  LineFollower.loop();

  int offset = LineFollower.getPositionOffset();
  static int ref = 190;  // reference signal

  int u = ref;  // control signal
  
  
  waitNextPeriod();

 
  Serial.println(offset);

  // get the new reference from the serial port is any.
  if (Serial.available() > 1) {  // something to read ?
    ref = Serial.parseInt();     // parse the value (ascii -> int)
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

  if (offset>=6 || offset <= -6){
    u = 40;
    LineFollower.setKp(0.1);
    pid_control_curve(u,offset);
    delay(30);
  }

  else {
    u = 120;
    LineFollower.setKp(0.04);
    pid_control(u,offset);
  }
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
  proportional = p_gain_curve * (error)*motor_speed/20; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*motor_speed/25;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit && motor_speed > 0) {
    setMotorAVoltage((motor_speed - delta_speed)*0.942);
    setMotorBVoltage(motor_speed + delta_speed);
  }
  last_error = error;
  //Serial.println(delta_speed);
  delay(50);
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