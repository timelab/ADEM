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

//#define I2CDEV_SERIAL_DEBUG

#ifdef DEBUG_GPS
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
//  if (I2Cdev::readBytes(i2cGpsAddress,I2C_GPS_REG_VERSION,1,(uint8_t *)&tmp)) {
  if (I2CGps::readBytes(i2cGpsAddress,I2C_GPS_REG_VERSION,1,(uint8_t *)&tmp)) {
    if ( tmp == 33) {
      Serial.println("I2C GPS OK, waiting for location fix");
      _initialized = true;
    } else {
      _initialized = false;
      Serial.print("I2C GPS no version match, version 0x");
      Serial.println(tmp, HEX);
    }
  } else {
    _initialized = false;
    Serial.print("No I2C (GPS) slave found at 0x");
    Serial.println(GPS_ADDRESS, HEX);
    delay(100); // give the I2C slave GPS a bit of time to initialize
  }
  yield();
}

void I2CGps::end() {
    _initialized = false;
    _measured = false;
    ready = false;
}

void I2CGps::write() {
}

void I2CGps::process() {
}

void I2CGps::read() {
  I2C_REGISTERS regs;
  if (!_initialized) I2CGps::begin();

//  if (_initialized && (I2Cdev::readBytes(i2cGpsAddress,0,32,(uint8_t *)&regs))) {
  if (_initialized && (I2CGps::readBytes(i2cGpsAddress,0,32,(uint8_t *)&regs) == 32)) {    
    /// TODO fill the structure byte by byte

    // use I2CDev library readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
    // I2Cdev::readbytes(GPS_ADDRESS,0,32,(uint8_t) &measuredData,0);

    measuredData.year = regs.year;
    measuredData.month = regs.month;
    measuredData.day = regs.day;
    measuredData.time = regs.time;
    measuredData.location = regs.gps_loc;
    measuredData.satellites = regs.status.numsats;
    measuredData.altitude = regs.altitude;
    measuredData.speed = regs.ground_speed;
    _measured = true;

    ready = (regs.status.gps2dfix == 1) && (measuredData.speed != 65535) && (measuredData.altitude != 65535) && (measuredData.location.lat != -1) && (measuredData.location.lon != -1) && (measuredData.location.lat != 0) && (measuredData.location.lon != 0) && (measuredData.month != 0) && (measuredData.day != 0);
    if (ready) {
      Serial.println("GPS READY");
      Serial.print("  lat="); Serial.print(measuredData.location.lat);
      Serial.print("  lon="); Serial.print(measuredData.location.lon);
      Serial.print("  gps2dfix="); Serial.print(regs.status.gps2dfix);
      Serial.print("  speed="); Serial.print(measuredData.speed);
      Serial.print("  altitude="); Serial.println( measuredData.altitude);
      
    } else {
      _measured = false;
      Serial.print("GPS waiting for fix, satellites="); Serial.println(regs.status.numsats);
      /* Serial.print("  gps2dfix="); Serial.println(regs.status.gps2dfix);
      Serial.print("  speed="); Serial.println(measuredData.speed);
      Serial.print("  altitude="); Serial.println( measuredData.altitude);
      Serial.print("  lat="); Serial.println(measuredData.location.lat);
      Serial.print("  lon="); Serial.println(measuredData.location.lon);
      */
//      delay (100); // give the GPS some time
      yield();
    }
  } else {
    _measured = false;
    ready = false;
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
//// replace I2CDEV readbytes for esp implementation
int8_t I2CGps::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();
    delay(10);
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission();
    yield();
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(" requestfrom ");
    #endif

    int8_t receiv_count = Wire.requestFrom(devAddr, (uint8_t)min(length , BUFFER_LENGTH));
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(receiv_count,DEC);
        Serial.print( " received ");
    #endif
    if (receiv_count >= min(length,BUFFER_LENGTH)) {
      for (; Wire.available(); count++) {
          data[count] = Wire.read();
          #ifdef I2CDEV_SERIAL_DEBUG
              Serial.print(data[count], HEX);
              if (count + 1 < length) Serial.print(" ");
          #endif
      }
    }
    // check for timeout
    if ( receiv_count == 0 ) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}
