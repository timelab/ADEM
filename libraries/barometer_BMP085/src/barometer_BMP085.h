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

// Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!

#ifndef _BMP085SENSOR_h
#define _BMP085SENSOR_h

#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <Wire.h>
#include <Arduino.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

struct bmp085Data : sensorData {
    short _temperature;
    long _pressure;
};


//abstract class Sensor
class BMP085Sensor : public Sensor{
public:
	//virtual function must be implemented
	virtual void begin();
	virtual void end();
	virtual void read();
	virtual void write();
	virtual void process();
	virtual String report();
    virtual String buildReport(sensorData *sData);
	//Sensor ();
	BMP085Sensor();
	void begin(uint8_t address);
    
private:
		
	const unsigned char OSS = 1;  // Oversampling Setting

	// Calibration values
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;

	// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
	// so ...Temperature(...) must be called before ...Pressure(...).
	long b5;

    bmp085Data measuredData;
	int8_t _sensor_address = BMP085_ADDRESS;

	// Use these for altitude conversions
	const float p0 = 101325;     // Pressure at sea level (Pa)
	float altitude;
	void Calibration();
	short GetTemperature(unsigned int ut);
	long GetPressure(unsigned long up);
	char ReadBMP(unsigned char address);
	int	ReadInt(unsigned char address);
	unsigned int ReadUT();
	unsigned long ReadUP();
};

#endif
