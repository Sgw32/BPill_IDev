#include "protocol.h"

extern int global_stop;
extern double desiredCur;
extern int obtainCurrentFromExternalDevice;
void parseSerial0()
{
  if (Serial.available() > 0) {  //если есть доступные данные
        // считываем байт
        desiredCur = (double)Serial.read();
        desiredCur /= 10;
        global_stop = 0;
  }
}

void parseSerial1()
{
    if (Serial1.available() > 0) {  //если есть доступные данные
        // считываем байт
        uint8_t temp = Serial.read();
        if ((temp==0xAB)&&(obtainCurrentFromExternalDevice==0))
        {
          obtainCurrentFromExternalDevice=1;
        }
        if (obtainCurrentFromExternalDevice)
        {
          uint8_t data1[8];
          while (temp!=0xFD)
          {
            
            temp = Serial.read();
          }
        }
  }
}
