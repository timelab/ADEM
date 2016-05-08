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

#ifndef _LipoBattery_h
#define _LipoBattery_h

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <..\..\Sensor\Sensor.h>


#ifdef DEBUG_BATTERY:
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

class LipoBattery : public Sensor{
public:
    LipoBattery();
    ~LipoBattery();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();

    boolean low = false;
    boolean empty = false;
    boolean charging = false;
    boolean full = false;

    uint16_t vcc;
};

#endif
