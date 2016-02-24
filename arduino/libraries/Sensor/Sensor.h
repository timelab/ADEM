/*
  Sensor.h - Skeleton Library for ADEM sensors.
  Created by Lieven Blancke, Koen Verstringe.

	Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/
#ifndef sensor_class_h
#define sensor_class_h

#include <Arduino.h>

//abstract class Sensor
class Sensor {
  public:
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void interrupt();
    //process function must be implemented
    virtual void process();
    virtual String report();
    //Sensor ();
};

#endif
