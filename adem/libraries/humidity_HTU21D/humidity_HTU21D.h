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

/***************************************************
  This is a library for the HTU21D-F Humidity & Temp Sensor

  Designed specifically to work with the HTU21D-F sensor from Adafruit
  ----> https://www.adafruit.com/products/1899

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
****************************************************/

#ifndef _HTU21DFSENSOR_h
#define _HTU21DFSENSOR_h
#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <Wire.h>
#include <Arduino.h>

#define HTU21DF_I2CADDR       0x40
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xE5
#define HTU21DF_WRITEREG       0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET       0xFE

struct HTU21DData : sensorData {
    float temperature;
    float humidity;
};

//abstract class Sensor
class HTU21DFSensor : public Sensor {
public:
	//virtual function must be implemented
	virtual void begin();
	virtual void end();
	virtual void read();
	virtual void write();
	virtual void process();
	virtual String report();
    virtual size_t dataBufferSize();	
    virtual String buildReport(sensorData *sData);
    virtual uint8_t * dataToBuffer();  
	//Sensor ();
	HTU21DFSensor();
	void begin(uint8_t address);
	void reset(void);
    
private:
	StaticJsonBuffer<200> jsonBuffer;

	const unsigned char OSS = 0;  // Oversampling Setting
    HTU21DData measuredData;
	boolean _error = false;
	boolean readData(void);

    int8_t _sensor_address = HTU21DF_I2CADDR;

	float GetTemperature();
	float GetHumidity();
	char ReadHTU21DF(unsigned char address);

};

#endif
