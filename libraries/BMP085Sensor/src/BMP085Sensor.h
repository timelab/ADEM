// BMP085Sensor.h
/*
Created by Lieven Blancke, Koen Verstringe.

Read https://www.arduino.cc/en/Reference/APIStyleGuide for inspiration!
*/

#ifndef _BMP085SENSOR_h
#define _BMP085SENSOR_h


#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <Wire.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

//abstract class Sensor
class BMP085Sensor {
public:
	//virtual function must be implemented
	virtual void begin();
	virtual void end();
	virtual void read();
	virtual void write();
	virtual void process();
	virtual String report();
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

	short _temperature;
	long _pressure;
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

