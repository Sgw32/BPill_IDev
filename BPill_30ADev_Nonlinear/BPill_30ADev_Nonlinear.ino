#include <OneWireSTM.h>

const double P1 = 1.0;
const double P2 = 0.02;
const double P3 = 0.001;
double desiredCur = 0.0f; // 38A at 65535 

const int analogInPin1 = PB0; 
const int analogInPin2 = PB1; 

const int pwmOutPin = PB9;    // PWM pin that the LED is attached to
double current = 0.0f;
double currentOffset = 0.0f;
double sensorValue1 = 0.0f;
double sensorValue2 = 0.0f;

double voltageModifier = 0.025; //0.3 V per 10A
double idleCurrent = 0.17; //A


float celsius, fahrenheit;
float startupCelsius = 0.0f;
float hCelsius = 1.0f;

double pwm = 0.0f;

HardwareTimer pwmtimer(4);
OneWireSTM  ds(PB9);  // on pin 10 (a 4.7K resistor is necessary)


#define PWM_OUT PA8       //PWM output
#define PWM_OUT_COMP PB13 //complementary output

#define FAN_CTL PB12 //complementary output


int temp_read_state = 0;  
int previousMillis = 0;        // will store the last time the was updated
int interval = 1000;            // interval at which to blink (in milliseconds)
byte addr[8];

int global_stop = 0;
int temp_ok = 1;

void prepareTemp(void)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];

  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }
   
  type_s = 0;
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  temp_read_state = 1;
  delay(750);
  //Serial.println("prepare state");
}

void readTemp(void) {
  //Serial.println("read state");
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  if (((float)raw / 16.0>-50.0f)&&((float)raw / 16.0<120.0f))
  {// check
    celsius = (float)raw / 16.0;
    temp_ok = 1;
  }
  else
    temp_ok = 0;
  temp_read_state = 0;    
}
  
void setupPWM()
{
  HardwareTimer timer1 = HardwareTimer(1);
 
  timer1.setPrescaleFactor(1);
  
  timer1.setPeriod(50); //10kHz

  
  pinMode(PWM_OUT, PWM); //Ideally these would be done as bit wise as I dont know exactly whats being set here..
  pinMode(PWM_OUT_COMP, PWM);

  
  timer_dev *t = TIMER1; //refers t to Timer 8 memory location, how to read back?
  timer_reg_map r = t->regs;
   
  bitSet(r.adv->CCER,0); //this should enable complimentary outputs
  bitSet(r.adv->CCER,2);
  //pwmWrite(PWM_OUT, 200); //
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB1 as an output.
  pinMode(analogInPin1, INPUT_ANALOG);
  pinMode(analogInPin2, INPUT_ANALOG);
  pinMode(PB3, OUTPUT);
  pinMode(pwmOutPin, PWM);
  pinMode(FAN_CTL, OUTPUT);
  digitalWrite(FAN_CTL, LOW);
  digitalWrite(PB3, LOW);
  delay(1000);
  //Calibration
  sensorValue1 = 3.3f/4095.0f*getADC(analogInPin1); //V1
  currentOffset = sensorValue1;
 // currentOffset = 1.20;
  current = 0;
  sensorValue1 = 0;

  setupPWM();
  
  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setPeriod(50);
  pwmWrite(PWM_OUT, 4095);
  pinMode(PC13, OUTPUT);
  digitalWrite(PB3, HIGH);
  Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
}

double getADC(int pin)
{
  int res=0;
  for (int i=0;i!=(1<<12);i++)
  {
    res+=(double)analogRead(pin);
  }
  return (double)(res>>12);
}

void testCurrent()
{
  if (global_stop)
    return;
  sensorValue1 += (3.314f/4095.0f*getADC(analogInPin1)-currentOffset-0.000858-sensorValue1)/10.0; //Volts
  current = sensorValue1/0.033; 
  
  sensorValue2 += (3.3f/4095.0f*getADC(analogInPin2)*2.0f+voltageModifier*current-sensorValue2)/1.5;

  float P = P1;
  if (fabs(desiredCur-current)<3.0f)
    P=P2;
  if (fabs(desiredCur-current)<0.5f)
    P=P3;
        
  pwm = pwm + P*(desiredCur-current);
  if ((pwm<0)||((current<0)||(fabs(desiredCur)<0.001)))
    pwm=0;
  if (pwm>4000.0)
    pwm=4000.0;

  if (((sensorValue2*3.765/3.65)<=2.5)&&(desiredCur>0)) //Voltage cut-off
  {
        desiredCur=0;
        pwm = 0;
        pwmWrite(PWM_OUT, 4095);
        global_stop = 1;
  }

  // thermal machine
  if (startupCelsius==0)
    startupCelsius = celsius; //write temperature at startup
  else
  {
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
  
  if ((millis() - previousMillis > (temp_read_state ? interval : 4000))||(temp_ok!=1)) {
        // Save the last time you blinked the LED
        previousMillis = millis();

        //if (temp_read_state==0)
          prepareTemp();
        //else
          readTemp();
    }
    
  testCurrent();
  pwmWrite(PWM_OUT, 4095-(int)map(pwm,0,4095.0f,0,4095.0f));
}
