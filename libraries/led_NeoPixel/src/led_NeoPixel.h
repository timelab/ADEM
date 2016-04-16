// led_NeoPixel.h
/*
Created by Dag Wieers, Stijn Diependaele

Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/

#ifndef _NeoPixelLed_h
#define _NeoPixelLed_h

#include <Arduino.h>
#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <Adafruit_NeoPixel.h>

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

    virtual void setbrightness(uint8_t);
    virtual void setcolor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
    virtual void setcolor(uint16_t n, uint32_t c);

    virtual uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

private:
    Adafruit_NeoPixel neopixel;

};

#endif
