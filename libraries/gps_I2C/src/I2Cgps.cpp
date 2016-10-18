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

#include "I2CGps.h"

#ifdef DEBUG_GPS:
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

I2CGps::I2CGps(int rx, int tx, int bd) {
    measuredData.ID = GPS_I2C;
}

I2CGps::I2CGps() {
    I2CGps(GPS_ADDRESS);
}

I2CGps::~I2CGps() {
    
}

void I2CGps::begin(void) {
    int tmp = 0;
    Serial.print("Initializing GPS... ");
    if (I2Cdev::readBytes(i2cGpsAddress,I2C_GPS_REG_VERSION,1,(uint8_t *)&tmp))
    {
        if ( tmp == 33)
           Serial.println("I2C GPS OK");
        else
           Serial.println("I2C GPS no version match");
    }
    else
    {
        Serial.print("No I2C (GPS) slave found at ");
        Serial.print(GPS_ADDRESS, HEX);
    }
}

void I2CGps::end() {
}

void I2CGps::write() {
}

void I2CGps::process() {
}

void I2CGps::read() {
    I2C_REGISTERS regs;
   
    if (I2Cdev::readBytes(i2cGpsAddress,0,32,(uint8_t *)&regs))
    {
    /// TODO fill the structure byte by byte
    
    // use I2CDev library readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
    // readbytes(GPS_ADDRESS,0,32,(uint8_t) &measuredData,0)

    measuredData.year = regs.year;
    measuredData.month = regs.month;
    measuredData.day = regs.day;
    measuredData.time = regs.time;
    measuredData.location = regs.gps_loc;
    measuredData.satellites = regs.status.numsats;
    measuredData.altitude = regs.altitude;
    measuredData.speed = regs.ground_speed;
	_measured = true;
    }
}

String I2CGps::FormatDateTime(I2CGPSData *date) {
    char gpsDateTime[26] = "";
    // time : hhmmssddd
    sprintf(gpsDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d0Z", date->year+2000, date->month, date->day, date->time/10000000,(date->time/100000)%100,(date->time/1000)%100,(date->time%1000));
    return gpsDateTime;
}

String I2CGps::report()  {
    if (!_measured)
        read();
    _measured = false;
    return buildReport( &measuredData);
}

String I2CGps::buildReport(sensorData *sData)  {

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    I2CGPSData *gpsData = reinterpret_cast<I2CGPSData *>(sData);
    
    root["Sensor"] = "GPS";

    root["Time"] = FormatDateTime(gpsData);
    
    root["Latitude"] = gpsData->location.lat;
    root["Longitude"] = gpsData->location.lon;
    
    root["Altitude"] = gpsData->altitude;
    
    root["Speed"] = gpsData->speed;
    
    root["Satellites"] = gpsData->satellites;
    
    root.printTo(response, sizeof(response));
    return response;
}