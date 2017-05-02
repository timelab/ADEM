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
 * Copyright 2016 Lieven Blancke, Koen Verstringe
 *
 */

/*
  Sensor.h - Skeleton Library for ADEM sensors.

  Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/
#ifndef sensor_class_h
#define sensor_class_h

#include <Arduino.h>
#include "SensorIDs.h"

struct sensorData{
    uint8_t ID;
};

//abstract class Sensor
class Sensor {
  public:
    Sensor();
    ~Sensor();
    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    uint8_t* bufferData();
    virtual String bufferedReport(uint8_t* buffer);
    virtual String report();
    virtual String buildReport(sensorData *sData);
    virtual size_t dataBufferSize();
    uint8_t * dataToBuffer();
    //Sensor ();
  protected:
    bool _measured = false;
    bool _initialized = false;
    sensorData measuredData;
};

#endif
