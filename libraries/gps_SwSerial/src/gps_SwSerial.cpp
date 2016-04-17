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

#include "gps_SwSerial.h"

#ifdef DEBUG_GPS:
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

SwSerialGPS::SwSerialGPS(int rx, int tx, int bd) {
    swserial = new SoftwareSerial(rx, tx, false, 512);
    tinygps = new TinyGPSPlus();
    baud = bd;
}

SwSerialGPS::SwSerialGPS() {
    SwSerialGPS(4, 0, 9600);
}

SwSerialGPS::~SwSerialGPS() {
    delete tinygps;
    delete swserial;
}

void SwSerialGPS::begin(void) {
    swserial->begin(baud);
}

void SwSerialGPS::end() {
}

void SwSerialGPS::write() {
}

void SwSerialGPS::process() {
}

void SwSerialGPS::read() {

    unsigned long start = millis();
    while (swserial->available() > 0 and millis() - start < 150) {
        char c = swserial->read();
        __LOG(c);
        tinygps->encode(c);
    }

    date = tinygps->date;
    time = tinygps->time;
    location = tinygps->location;
    satellites = tinygps->satellites;
}

String SwSerialGPS::FormatDateTime(TinyGPSDate date, TinyGPSTime time) {
    char gpsDateTime[26] = "";
    sprintf(gpsDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d0Z", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second(), time.centisecond());
    return gpsDateTime;
}

String SwSerialGPS::report()  {

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    root["Sensor"] = "GPS";

    if (tinygps->date.isValid() && tinygps->time.isValid()) {
        root["Time"] = FormatDateTime(date, time);
    }

    if (tinygps->location.isValid()) {
        ready = true;
        root["Lattitude"] = location.lat();
        root["Longitude"] = location.lng();
    } else {
        ready = false;
    }

    if (tinygps->satellites.isValid()) {
        root["Satellites"] = satellites.value();
    }

    root.printTo(response, sizeof(response));
    return response;
}
