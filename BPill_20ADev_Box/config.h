#ifndef CONFIG_H
#define CONFIG_H

#define PWM_OUT PA8       //PWM output
#define PWM_OUT_COMP PB13 //complementary output
#define FAN_CTL PB12 //complementary output

HardwareTimer pwmtimer(4);
//OneWireSTM  ds(PB9);  // on pin 10 (a 4.7K resistor is necessary)


#endif