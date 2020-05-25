#ifndef TSENSOR_H
#define TSENSOR_H

#include <OneWireSTM.h>
#include <DS18B20.h>

// 1-Wire bus Arduino pin
#define ONEWIRE_PIN PB9
// Number of sensors
#define SENSORS_NUM 5

void initTSensor(void);
void prepareTemp(void);
void obtainAddress(void);
void readTemp(void);

void readSensors(void);

#endif
