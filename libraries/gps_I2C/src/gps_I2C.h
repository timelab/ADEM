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
 * Copyright 2016 Dag Wieers, Lieven Blancke
 *
 */

#ifndef _gps_I2C_h
#define _gps_I2C_h
#include <Arduino.h>
#include <Sensor.h>
#include <ArduinoJson.h>
#include <wire.h>

#include "i2c_gps_registers.h"                 //Register definitions

#define GPS_ADDRESS 20


//abstract class Sensor
class GPS_I2C : public Sensor {
public:
    GPS_I2C(int rx=4, int tx=0, int bd=9600);
    GPS_I2C();
    ~GPS_I2C();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();
    virtual String buildReport(sensorData *sData);
    
    boolean ready = false;

private:
    StaticJsonBuffer<200> jsonBuffer;
    
    I2CGPSData measuredData;
    String FormatDateTime(uint32_t time);

};

#endif
