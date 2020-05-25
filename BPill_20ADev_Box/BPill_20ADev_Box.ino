#include "TSensor.h"
#include "ADC.h"
#include "pwm.h"
#include "config.h"
#include <OneWireSTM.h>

const double P = 10.0;
const double I = 0.01;
const double D = 00.0;

double desiredCur = 0.0f; // 38A at 65535 
const int analogInPin1 = PB0; 
const int analogInPin2 = PB1; 
double current = 0.0f;
double prev_current = 0.0f;

double currentOffset = 0.0f;
double sensorValue1 = 0.0f;
double sensorValue2 = 0.0f;
double voltageModifier = 0.025; //0.3 V per 10A
double idleCurrent = 0.17; //A

double pwm = 0.0f;
int previousMillis = 0;        // will store the last time the was updated
int interval = 1000;            // interval at which to blink (in milliseconds
int global_stop = 0;

//#define CHECK_CUTOFF_VOLTAGE
//#define CHECK_TEMPERATURE_HYSTERESIS

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB1 as an output.
  pinMode(analogInPin1, INPUT_ANALOG);
  pinMode(analogInPin2, INPUT_ANALOG);
  pinMode(PB3, OUTPUT);
  pinMode(FAN_CTL, OUTPUT);
  digitalWrite(FAN_CTL, LOW);
  digitalWrite(PB3, LOW);
  //delay(1000);
  //Calibration
   for (int i=0;i!=256;i++)
    sensorValue1 += 3.3f/4095.0f*getADC(analogInPin1); //V1
  sensorValue1/=256.0f;
  currentOffset = sensorValue1;
 // currentOffset = 1.20;
  current = 0;
  sensorValue1 = 0;

  setupPWM();
  obtainAddress();
  initTSensor();
  
  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setPeriod(50);
  pwmWrite(PWM_OUT, 4095);
  pinMode(PC13, OUTPUT);
  digitalWrite(PB3, HIGH);
  Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
}

void testCurrent()
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

// the loop function runs over and over again forever
void loop() {
  if (Serial.available() > 0) {  //если есть доступные данные
        // считываем байт
        desiredCur = (double)Serial.read();
        desiredCur /= 10;
        global_stop = 0;
  }

   if (sensors.available())
  {
    // Reads the temperature from sensor
    celsius = sensors.readTemperature(addr);
    if ((celsius>-50.0f)&&(celsius<120.0f))
    {// check
      temp_ok = 1;
    }
    else
      temp_ok = 0;
    temp_read_state = 0;  
    // Another requests sensor for measurement
    sensors.request(addr);   
  }
    
  testCurrent();
  pwmWrite(PWM_OUT, 4095-(int)map(pwm,0,4095.0f,0,4095.0f));
}
