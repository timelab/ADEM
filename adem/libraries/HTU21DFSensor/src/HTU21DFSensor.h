// HTU21DFSensor.h

#ifndef _HTU21DFSENSOR_h
#define _HTU21DFSENSOR_h
#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <Wire.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



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

#include "Wire.h"

#define HTU21DF_I2CADDR       0x40
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xE5
#define HTU21DF_WRITEREG       0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET       0xFE

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
	
	//Sensor ();
	HTU21DFSensor();
	void begin(uint8_t address);
	void reset(void);

private:
	StaticJsonBuffer<200> jsonBuffer;

	const unsigned char OSS = 0;  // Oversampling Setting

	boolean _error = false;
	boolean readData(void);
	float humidity, temperature;
	int8_t _sensor_address = HTU21DF_I2CADDR;

	float GetTemperature();
	float GetHumidity();
	char ReadHTU21DF(unsigned char address);

};

#endif
