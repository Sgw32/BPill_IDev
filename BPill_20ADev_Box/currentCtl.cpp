#include "currentCtl.h"

extern int global_stop;
extern double sensorValue1;
extern double sensorValue2;
extern double currentOffset;
extern double current;
extern double desiredCur;
extern double prev_current;
extern double voltageModifier;
extern double idleCurrent;
extern double pwm;

extern float celsius; // Температура
extern float startupCelsius; //значение Т при включении
extern float hCelsius;

const double P = 10.0;
const double I = 0.01;
const double D = 00.0;

void processCycle()
{
  if (global_stop)
    return;
  sensorValue1 += (3.3f/4095.0f*getADC(analogInPin1)-currentOffset-sensorValue1)/10.0; //Volts
  current = sensorValue1/0.033; 
  double delta_current = prev_current-current;
  prev_current = current;
  
  sensorValue2 += (3.3f/4095.0f*getADC(analogInPin2)*2.0f+voltageModifier*current-sensorValue2)/1.5;
  
  //P(I)D
  pwm = pwm + P*(desiredCur-current)+D*delta_current/10.0f;
  if ((pwm<0)||((current<0)||(fabs(desiredCur)<0.001)))
    pwm=0;
  if (pwm>4000.0)
    pwm=4000.0;

#ifdef CHECK_CUTOFF_VOLTAGE
  if (((sensorValue2*3.765/3.65)<=2.5)&&(desiredCur>0)) //Voltage cut-off
  {
        desiredCur=0;
        pwm = 0;
        pwmWrite(PWM_OUT, 4095);
        global_stop = 1;
  }
#endif

  // thermal machine

  if (startupCelsius==0)
    startupCelsius = celsius; //write temperature at startup
  else
  {
#ifdef CHECK_TEMPERATURE_HYSTERESIS
    if (fabs(startupCelsius-celsius)>hCelsius) //if it heated more than 1 C
    {
      pwm=0; //stop current flow
      hCelsius = 0.5f; //setup hysteresys;
      digitalWrite(FAN_CTL, HIGH);
    }
    else
    { 
      hCelsius = 1.0f;
      digitalWrite(FAN_CTL, LOW);
    }
#endif
  }


  Serial.print(millis());
  Serial.print(" \t t \t");
  Serial.print(sensorValue2*3.765/3.65);
  Serial.print(" \t V \t");
  Serial.print(currentOffset);
  Serial.print(" \t A \t");
  Serial.print(desiredCur);
  Serial.print(" \t A \t");
  Serial.print(celsius);
  Serial.print(" \t *C \t");
  Serial.print(startupCelsius);
  Serial.print(" \t *C \t");
  Serial.print(hCelsius);
  Serial.print(" \t *C \t");
  Serial.print(fabs(startupCelsius-celsius));
  Serial.print(" \t *C \t");
  Serial.print(pwm);
  Serial.print(" \t");
  Serial.print(current);
  Serial.print(" A\n");
}
