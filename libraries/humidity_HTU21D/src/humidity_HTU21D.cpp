// 
// 
// 

#include "humidity_HTU21D.h"

/***************************************************
This is a library for the HTU21DF Humidity & Temp Sensor
Designed specifically to work with the HTU21DF sensor from Adafruit
----> https://www.adafruit.com/products/1899
These displays use I2C to communicate, 2 pins are required to
interface
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution
****************************************************/

#if defined(__AVR__)
#include <util/delay.h>
#endif

HTU21DFSensor::HTU21DFSensor() {
}


void HTU21DFSensor::begin(void) {
	begin(HTU21DF_I2CADDR);
}

void HTU21DFSensor::begin(uint8_t address) {
	Wire.begin();
	_sensor_address = address;
	reset();

	Wire.beginTransmission(_sensor_address);
	Wire.write(HTU21DF_READREG);
	Wire.endTransmission();
	Wire.requestFrom(_sensor_address, 1);
	_error = (Wire.read() == 0x2); // after reset should be 0x2

}

void HTU21DFSensor::reset(void) {
	Wire.beginTransmission(_sensor_address);
	Wire.write(HTU21DF_RESET);
	Wire.endTransmission();
	delay(15);
	_error = false;
}


void HTU21DFSensor::end() {
}

void HTU21DFSensor::write() {
}

void HTU21DFSensor::process() {
}

float HTU21DFSensor::GetTemperature(void) {

	// OK lets ready!
	Wire.beginTransmission(_sensor_address);
	Wire.write(HTU21DF_READTEMP);
	Wire.endTransmission();

	delay(50); // add delay between request and actual read!
	Wire.requestFrom(_sensor_address, 3);
	while (!Wire.available()) {}

	uint16_t t = Wire.read();
	t <<= 8;
	t |= Wire.read();

	uint8_t crc = Wire.read();
	float temp = t;
	temp *= 175.72;
	temp /= 65536;
	temp -= 46.85;
	
	temperature = temp;
	return temp;
}


float HTU21DFSensor::GetHumidity(void) {
	// OK lets ready!
	Wire.beginTransmission(_sensor_address);
	Wire.write(HTU21DF_READHUM);
	Wire.endTransmission();

	delay(50); // add delay between request and actual read!

	Wire.requestFrom(_sensor_address, 3);
	while (!Wire.available()) {}

	uint16_t h = Wire.read();
	h <<= 8;
	h |= Wire.read();

	uint8_t crc = Wire.read();

	float hum = h;
	hum *= 125;
	hum /= 65536;
	hum -= 6;
	humidity = hum;
	return hum;
}

void HTU21DFSensor::read() {
	GetHumidity();
	GetTemperature();
}

String HTU21DFSensor::report()  {
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	char response[200];
	root["Sensor"] = "HTU21DF";
	root["Temperature"] = temperature;
	root["Humidity"] = humidity;
	root.printTo(response,sizeof(response));
	return response;
}
