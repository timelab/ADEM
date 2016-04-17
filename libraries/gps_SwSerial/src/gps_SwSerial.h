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

#ifndef _SwSerialGPS_h
#define _SwSerialGPS_h
#include <Arduino.h>
#include <Sensor.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPS_RX_PIN 4
#define GPS_TX_PIN 0

#define GPS_BAUD 9600

//abstract class Sensor
class SwSerialGPS : public Sensor {
public:
    SwSerialGPS(int rx=4, int tx=0, int bd=9600);
    SwSerialGPS();
    ~SwSerialGPS();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();

    boolean ready = false;

private:
    StaticJsonBuffer<200> jsonBuffer;

    TinyGPSDate date;
    TinyGPSTime time;
    TinyGPSLocation location;
    TinyGPSInteger satellites;
    TinyGPSAltitude altitude;
    TinyGPSSpeed speed;
    String FormatDateTime(TinyGPSDate date, TinyGPSTime time);

    int baud = 9600;

    SoftwareSerial *swserial;
    TinyGPSPlus *tinygps;
};

#endif
