#include "adc.h"

double getADC(int pin)
{
  int res=0;
  for (int i=0;i!=(1<<12);i++)
  {
    res+=(double)analogRead(pin);
  }
  return (double)(res>>12);
}