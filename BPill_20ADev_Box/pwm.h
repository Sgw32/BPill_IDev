#ifndef PWM_H
#define PWM_H

#include "config.h"

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

#endif