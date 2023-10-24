#include "Motors.h"
#import <Arduino.h>

void InitMotors()
{
 pinMode(A_BI1,OUTPUT);
 pinMode(A_BI2,OUTPUT);
 pinMode(A_PWM,OUTPUT);
 pinMode(B_PWM,OUTPUT);
 pinMode(B_BI1,OUTPUT);
 pinMode(B_BI2,OUTPUT);
}


// fonction permettant de gerer l'alimentation moteur (sens et amplitude)
void setMotorAVoltage(int valeur)
{
  if(valeur<0)
  {
    digitalWrite(A_BI1,1);
    digitalWrite(A_BI2,0);
  }
  else
   {
    digitalWrite(A_BI1,0);
    digitalWrite(A_BI2,1);
  }
  analogWrite(A_PWM,constrain(abs(valeur),0,MAXPWM));
}

// fonction permettant de gerer l'alimentation moteur (sens et amplitude)
void setMotorBVoltage(int valeur)
{
  if(valeur<0)
  {
    digitalWrite(B_BI1,1);
    digitalWrite(B_BI2,0);
  }
  else
   {
    digitalWrite(B_BI1,0);
    digitalWrite(B_BI2,1);
  }
  analogWrite(B_PWM,constrain(abs(valeur),0,MAXPWM));
}

// Le temps de décélération est réglage avec le changement du delay

