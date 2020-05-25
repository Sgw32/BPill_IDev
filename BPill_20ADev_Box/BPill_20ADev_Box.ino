#include "TSensor.h"
#include "adc.h"
#include "pwm.h"
#include "config.h"
#include <OneWireSTM.h>
#include "protocol.h"
#include "currentCtl.h"

double desiredCur = 0.0f; // 38A at 65535 
double current = 0.0f;
double prev_current = 0.0f;

double currentOffset = 0.0f;
double sensorValue1 = 0.0f;
double sensorValue2 = 0.0f;
double voltageModifier = 0.025; //0.3 V per 10A
double idleCurrent = 0.17; //A

HardwareTimer pwmtimer(4);

double pwm = 0.0f;
int previousMillis = 0;        // will store the last time the was updated
int interval = 1000;            // interval at which to blink (in milliseconds
int global_stop = 0;

int obtainCurrentFromExternalDevice = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB1 as an output.
  setupGPIO();
  delay(1000);
  //Calibration
   for (int i=0;i!=256;i++)
    sensorValue1 += 3.3f/4095.0f*getADC(analogInPin1); //V1
  sensorValue1/=256.0f;
  currentOffset = sensorValue1;
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
  Serial1.begin(9600);
}

// the loop function runs over and over again forever
void loop() 
{
  parseSerial0();
  parseSerial1();
  readSensors();
  processCycle();
  pwmWrite(PWM_OUT, 4095-(int)map(pwm,0,4095.0f,0,4095.0f));
}
