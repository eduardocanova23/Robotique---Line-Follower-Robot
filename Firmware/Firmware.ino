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

MeUltrasonicSensor ultraSensor(PORT_6);
//MeUltrasonicSensor ultraSensor2(PORT_7);
//MeCompass MeCompass(PORT_7, ADDRESS3);
MeRGBLineFollower LineFollower(PORT_8);

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
int statezerocount = 0;

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

void pid_control_curveA(int motor_speed, int sensor_value) {
  //setMotorAVoltage(0);
  //setMotorBVoltage(0);
  int error = sensor_value; // between -512 and 512
  proportional = p_gain_curve * (error)*motor_speed/30; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*motor_speed/35;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit) {
    setMotorAVoltage((motor_speed - delta_speed)*0.942);
    
  }
  last_error = error;
  //Serial.println(delta_speed);
  //delay(100);
}

void pid_control_curveB(int motor_speed, int sensor_value) {
  //setMotorAVoltage(0);
  //setMotorBVoltage(0);
  int error = sensor_value; // between -512 and 512
  proportional = p_gain_curve * (error)*motor_speed/30; // replace 10 by max value of steering fct
  integral += (i_gain * error)/500;
  derivative = d_gain * (error - last_error)*motor_speed/35;

  float delta_speed = (proportional + derivative + integral);
  if (motor_speed + delta_speed < speed_limit) {
    setMotorBVoltage((motor_speed + delta_speed));
    
  }
  last_error = error;
  //Serial.println(delta_speed);
  //delay(100);
}

void setup() {
  

  //gyro.begin();
  LineFollower.begin();
  kp = 0.04;
  LineFollower.setKp(kp);

  // initialization of the serial communication.
  Serial.begin(9600);
  Serial.setTimeout(10);


}

void loop() {
  // Main loop
  float dist = ultraSensor.distanceCm();
  //float dist2 = ultraSensor2.distanceCm();
  
  if (abs(millis()-lastLock) > 1000){
    flagCurveLock = true;
  }
  LineFollower.loop();
  int offset = LineFollower.getPositionOffset();
  //gyro.update();
  
  static int ref = 190;  // reference signal
  //ang = gyro.getAngleZ();
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
    statezerocount ++;
    LineFollower.setKp(0.04);
    LineFollower.loop();
    int offset = LineFollower.getPositionOffset();
    Serial.println(offset);
    //gyro.update();
    //ang = gyro.getAngleZ();

    //dist = ultraSensor.distanceCm();
    if (statezerocount % 3 == 0){
      dist = ultraSensor.distanceCm();
      Serial.println(dist);
    }

    

    pid_control(100,offset);

    if (abs(millis()-lastLock) > 1000){
      // if some time passed since last lock, unlock the curve
      flagCurveLock = true;
    }

    if ((offset>=13 || offset <= -13) && flagCurveLock){
      state = 1; // starting to detect the curve -> change the state to curve
      Serial.println("ENTREI AQUI");
    }
/*
    if (dist2 < 1){
      state = 3; // detected obstacle -> change the state to wait
    }
*/
    if (dist < 20){
      state = 2; // detected obstacle -> change the state to dodge
    }
  }

  // ==========================================================================================
  // ============================ FIM DO ESTADO 0 =============================================
  // ==========================================================================================

  //lastang = gyro.getAngleZ();
  float timebeforecurve = millis();
  delay(30);

  while (state == 1 && flagCurveLock){
    Serial.println(millis() - timebeforecurve);
    LineFollower.loop();
    LineFollower.setKp(0.08);
    offset = LineFollower.getPositionOffset();
    //Serial.println(offset);
    //dist = ultraSensor.distanceCm();
    //dist2 = ultraSensor2.distanceCm();
    //Serial.println(offset);
    //Serial.println("CURVA");
    //gyro.update();
    //ang = gyro.getAngleZ();
    
    if (offset < 0){
      pid_control_curveA(25, offset);
      pid_control_curveB(-20, offset);
      //pid_control_curve(-25,25,offset);
      //pid_control_dodge(35, offset);
    }

    else{
      //pid_control_dodge(35, offset);
      pid_control_curveA(-20, offset);
      pid_control_curveB(25, offset);
      //pid_control_curve(25,-25,offset);
    }
    

     
    if (abs(millis() - timebeforecurve) >= 1100){ 
      Serial.println("SAI DA curva");
      state = 0;
      flagCurveLock = false;
      lastLock = millis();
    }


    if (dist < 25){
      state = 2; // detected obstacle -> change the state to dodge
    }
  }


  
  if (state == 2){

    LineFollower.loop();
    int offset = LineFollower.getPositionOffset();
    Serial.println(dist);
    

    //Serial.println("DESVIA");
      
    setMotorBVoltage(-40);
    setMotorAVoltage(-40*0.942);
    delay(600);

    

    setMotorBVoltage(0);
    setMotorAVoltage(50);
    delay(700);
    
    setMotorBVoltage(50);
    setMotorAVoltage(50*0.942);
    delay(200);

    setMotorBVoltage(50);
    setMotorAVoltage(0);
    delay(700);
    
    setMotorBVoltage(50);
    setMotorAVoltage(50*0.942);
    delay(650);

    setMotorBVoltage(50);
    setMotorAVoltage(0);
    delay(800);

    setMotorBVoltage(50);
    setMotorAVoltage(50*0.942);
    delay(300);

    setMotorBVoltage(0);
    setMotorAVoltage(50);
    delay(850);

    state =1;
  }
  
  while (state == 3){

    //Serial.println("ESPERA");
    //dist2 = ultraSensor2.distanceCm();
    dist = ultraSensor.distanceCm();
    setMotorBVoltage(0);
    setMotorAVoltage(0.942*0);
    //if (dist2 > 40){
      //state == 0;
    //}
    
  }
}
