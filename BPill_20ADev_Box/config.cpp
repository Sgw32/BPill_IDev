#include "config.h"

void setupGPIO()
{
  pinMode(analogInPin1, INPUT_ANALOG);
  pinMode(analogInPin2, INPUT_ANALOG);
  pinMode(PB3, OUTPUT);
  pinMode(FAN_CTL, OUTPUT);
  digitalWrite(FAN_CTL, LOW);
  digitalWrite(PB3, LOW);
}