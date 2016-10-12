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
 * Copyright 2016 Dag Wieers, Lieven Blancke, Koen Verstringe
 *
 */

#include "gps_I2C.h"

#ifdef DEBUG_GPS:
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

GPS_I2C::GPS_I2C(int rx, int tx, int bd) {
    measuredData.ID = GPS_I2C;
}

GPS_I2C::GPS_I2C() {
    GPS_I2C(GPS_ADDRESS);
}

GPS_I2C::~GPS_I2C() {
    
}

void GPS_I2C::begin(void) {
    Serial.print("Initializing GPS... ");
    Wire.beginTransmission(GPS_ADDRESS);
    error = Wire.endTransmission();
    if (error == 0)
    {
        Wire.beginTransmission(GPS_ADDRESS);
        Wire.write(I2C_GPS_REG_VERSION);
        error = Wire.endTransmission();
        Wire.requestFrom(GPS_ADDRESS, 1);
        uint8_t c = Wire.read();
        if ( c == 22)
           Serial.println("I2C GPS OK");
        else
           Serial.println("I2C GPS no version match");
    }
    else
    {
        Serial.print("No I2C GPS found at ");
        Serial.print(GPS_ADDRESS, HEX);
    }
}

void GPS_I2C::end() {
}

void GPS_I2C::write() {
}

void GPS_I2C::process() {
}

void GPS_I2C::read() {

    unsigned long start = millis();
    while (swserial->available() > 0 and millis() - start < 150) {
        char c = swserial->read();
        __LOG(c);
        tinygps->encode(c);
    }
    Wire.beginTransmission(GPS_ADDRESS);
    Wire.write(I2C_GPS_REG_VERSION);
    error = Wire.endTransmission();
    Wire.requestFrom(GPS_ADDRESS, 20);
    uint8_t c = Wire.read();
    /// TODO fill the structure byte by byte
    
    // use I2CDev library readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
    // readbytes(GPS_ADDRESS,0,32,(uint8_t) &measuredData,0)
    // tot nu toe hebben we nog geen date in de I2C gps. registers zijn voorzien maar we krijgen de data nog niet
    measuredData.date = tinygps->date;
    measuredData.time = tinygps->time;
    measuredData.location = tinygps->location;
    measuredData.satellites = tinygps->satellites;
    measuredData.altitude = tinygps->altitude;
    measuredData.speed = tinygps->speed;
	_measured = true;
}

String GPS_I2C::FormatDateTime(TinyGPSDate date, TinyGPSTime time) {
    char gpsDateTime[26] = "";
    sprintf(gpsDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d0Z", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second(), time.centisecond());
    return gpsDateTime;
}

String GPS_I2C::report()  {
    if (!_measured)
        read();
    _measured = false;
    return buildReport( &measuredData);
}

String GPS_I2C::buildReport(sensorData *sData)  {

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    GPS_I2CData *gpsData = reinterpret_cast<GPS_I2CData *>(sData);
    
    root["Sensor"] = "GPS";

    if (gpsData->date.isValid() && gpsData->time.isValid()) {
        root["Time"] = FormatDateTime(gpsData->date,gpsData->time);
    }

    if (gpsData->location.isValid()) {
        root["Latitude"].set(gpsData->location.lat(), 5);
        root["Longitude"].set(gpsData->location.lng(), 5);
        ready = true;
    } else {
        ready = false;
    }

    if (gpsData->altitude.isValid()) {
        root["Altitude"] = gpsData->altitude.meters();
    }

    if (gpsData->speed.isValid()) {
        root["Speed"] = gpsData->speed.kmph();
    }

    if (gpsData->satellites.isValid()) {
        root["Satellites"] = gpsData->satellites.value();
    }

    root.printTo(response, sizeof(response));
    return response;
}
