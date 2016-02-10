/*
  Sensor.h - Skeleton Library for ADEM sensors.
  Created by Lieven Blancke, Koen Verstringe.

	Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/
#ifndef sensor_class_h
#define sensor_class_h

#include "Arduino.h"

class Sensor {
public:
    void begin();
    void end();
    void read();
    void write();
    void interrupt();
    void process();
    String report(); // should report a JSON string
private:
    uint _size;
};

#endif
