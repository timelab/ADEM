/*
 * This file is part of the ADEM project.
 *
 * ADEM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,·
 * (at your option) any later version.
 *
 * ADEM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ADEM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 Dag Wieers, Stijn Diependaele
 *
 */

#ifndef _NeoPixelLed_h
#define _NeoPixelLed_h

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class NeoPixelLed {
public:
    NeoPixelLed(uint16_t n, uint8_t p=5, neoPixelType t=NEO_GRB + NEO_KHZ800);
    NeoPixelLed();
    ~NeoPixelLed();

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
