/*
  Ppd42.h - Library for ADEM PPD42 sensor.
  Created by Lieven Blancke.
*/

#ifndef Ppd42_h
#define Ppd42_h

#include <Arduino.h>
#include <Sensor.h>

class Ppd42 : public Sensor {
  public:
    void begin();
    void end();
    void read();
    void write();
    void interrupt();
    void process();
    Ppd42();
    String report(); // should report a JSON string

  private:
 //   uint _particleCount;
};

#endif
