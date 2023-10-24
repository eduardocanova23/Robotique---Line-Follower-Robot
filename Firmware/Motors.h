// PIN utilisées pour l'alimentation moteur
#define A_PWM 12
#define B_PWM 8
#define A_BI1 35
#define A_BI2 34
#define B_BI1 37
#define B_BI2 36

#define MAXPWM 190 // maximum duty cycle for the PWM is 255/MAXPWM

void InitMotors();

void setMotorAVoltage(int value);
void setMotorBVoltage(int value);