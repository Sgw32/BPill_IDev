#ifndef TSENSOR_H
#define TSENSOR_H

#include <OneWireSTM.h>
#include <DS18B20.h>

// 1-Wire bus Arduino pin
const byte ONEWIRE_PIN = PB9;

// Number of sensors
const byte SENSORS_NUM = 5;

int temp_ok = 1;
byte addr[8];

// Sensors address
// EXAMPLE:
// const byte sensorsAddress[SENSORS_NUM][8] PROGMEM = {
//     0x28, 0xB1, 0x6D, 0xA1, 0x3, 0x0, 0x0, 0x11,
//     0x28, 0x87, 0x6A, 0xA1, 0x3, 0x0, 0x0, 0x1F
// };

const byte sensorsAddress[SENSORS_NUM][8] PROGMEM = {
    0x28 ,0x3A ,0x4D ,0x54 ,0xA ,0x0 ,0x0 ,0x85 ,
    0x28 ,0x3E ,0x64 ,0x54 ,0xA ,0x0 ,0x0 ,0x52 ,
    0x28 ,0xE1 ,0x7D ,0x53 ,0xA ,0x0 ,0x0 ,0xBB ,
    0x28 ,0xD1 ,0xC2 ,0x52 ,0xA ,0x0 ,0x0 ,0xF9 ,
    0x28 ,0x6F ,0xD5 ,0x53 ,0xA ,0x0 ,0x0 ,0x4F
};

OneWire ds(ONEWIRE_PIN); // 1-Wire object
DS18B20 sensors(&ds); // DS18B20 sensors object
int temp_read_state = 0;  

float celsius; // Температура
float startupCelsius = 0.0f; //значение Т при включении
float hCelsius = 1.0f; //Гистерезис на переключение по датчику Т

void initTSensor()
{
    // DS18B20 sensors setup
  sensors.begin();
    
  // The first requests to all sensors for measurement
  sensors.request();
}

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
}

void obtainAddress(void)
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
  
  delay(750);
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

#endif
