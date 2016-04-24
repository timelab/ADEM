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

#include "humidity_HTU21D.h"

#if defined(__AVR__)
#include <util/delay.h>
#endif

HTU21DFSensor::HTU21DFSensor() {
    measuredData.ID = HUMIDITY_HTU21D;
}

void HTU21DFSensor::begin(void) {
	Serial.print("Initializing humidity sensor... ");
	begin(HTU21DF_I2CADDR);
	Serial.println("OK");
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
	return hum;
}

void HTU21DFSensor::read() {
    measuredData.humidity = GetHumidity();
    measuredData.temperature = GetTemperature();
    measuredData.measured = true;
}

String HTU21DFSensor::report()  {
    if (!measuredData.measured)
        read();
    measuredData.measured = false;
    return buildReport(&measuredData);
}

String HTU21DFSensor::buildReport(sensorData *sData)  {
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	char response[200];
    HTU21DData *htuData = reinterpret_cast<HTU21DData *>(sData);
	root["Sensor"] = "HTU21DF";
	root["Temperature"] = htuData->temperature;
	root["Humidity"] = htuData->humidity;
	root.printTo(response,sizeof(response));
	return response;
}
