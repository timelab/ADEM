// led_NeoPixel.h
/*
Created by Dag Wieers, Stijn Diependaele

Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/

#ifndef _LED_h
#define _LED_h


#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <Adafruit_NeoPixel.h>

#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#else
    #include "WProgram.h"
#endif

//abstract class Sensor
class NeoPixelLed {
public:
    NeoPixelLed(uint16_t n, uint8_t p=5, neoPixelType t=NEO_GRB + NEO_KHZ800);
    NeoPixelLed();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();
    //Sensor ();
    virtual void setbrightness(uint8_t);
    virtual void setcolor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);

private:

    Adafruit_NeoPixel neopixel;

};

#endif
