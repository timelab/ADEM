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
 * Copyright 2016 Lieven Blancke
 *
 */

/*
  Ppd42.cpp - Library for ADEM PPD42 dust sensor.

  Connect PPD42 PM1.0 pin to Arduino pin 12
  Connect PPD42 PM2.5 pin to Arduino pin 13

  XXX: This device does not measure PM10, but PM1.0 !!
*/

#include <Arduino.h>
#include <particulate_PPD42.h>

/*
	instantiate and initialize the static variables that are used in 
	the static interrupt functions that handle the hardware interrupts 
	triggered by the PPD42 dust sensor
*/
int PPD42Sensor::_PPD_PM10_PIN = 12;
int PPD42Sensor::_PPD_PM25_PIN = 13;
volatile unsigned long PPD42Sensor::_triggerStartMicrosPM10 = 0;
volatile unsigned long PPD42Sensor::_triggerStartMicrosPM25 = 0;
volatile unsigned long PPD42Sensor::_triggeredTotalMicrosPM25 = 0;
volatile unsigned long PPD42Sensor::_triggeredTotalMicrosPM10 = 0;

PPD42Sensor::PPD42Sensor(){
    measuredData.ID = PARTICULATE_PPD42;
}

void PPD42Sensor::begin() {
	Serial.print("Initializing particulate sensor... ");
	_measured = false;

	// initialization for the dust sensor
	_readMillisPM10 = millis(); //Fetch the current time
	_readMillisPM25 = millis(); //Fetch the current time
	_triggeredTotalMicrosPM10 = 0;
	_triggeredTotalMicrosPM25 = 0;

	pinMode(_PPD_PM10_PIN, INPUT);
	pinMode(_PPD_PM25_PIN, INPUT);
	_activated = true;
	
	// Attach interrupts handlers to hardware pins
	attachInterrupt(digitalPinToInterrupt(_PPD_PM10_PIN), _handleInterruptPM10, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_PPD_PM25_PIN), _handleInterruptPM25, CHANGE);

	Serial.println("OK");
}

void PPD42Sensor::end() {
	_activated = false;
	// detach the interrupts
	detachInterrupt(_PPD_PM10_PIN);
	detachInterrupt(_PPD_PM25_PIN);
}

void PPD42Sensor::read() {
    measuredData.PM10Ppm = readPM10Ppm();
    measuredData.PM25Ppm = readPM25Ppm();
    _measured = true;
}

unsigned long PPD42Sensor::readPM10Ppm() {
	if (_activated) {
		unsigned long _currentMillis=millis(); 
		unsigned long _sampledMillis = _currentMillis - _readMillisPM10;
		unsigned long _triggeredTotalMicros = _triggeredTotalMicrosPM10;
		_readMillisPM10 = _currentMillis;
		_triggeredTotalMicrosPM10 = 0;
		return 1000 * _triggeredTotalMicros / _sampledMillis;
	}
	else
		return 0;
}

unsigned long PPD42Sensor::readPM25Ppm() {
	if (_activated) {
		unsigned long _currentMillis=millis(); 
		unsigned long _sampledMillis = _currentMillis - _readMillisPM25;
		unsigned long _triggeredTotalMicros = _triggeredTotalMicrosPM25;
		_readMillisPM25 = _currentMillis;
		_triggeredTotalMicrosPM25 = 0;
		return 1000 * _triggeredTotalMicros / _sampledMillis;
	}
	else
		return 0;
}

void PPD42Sensor::write() {
}

void PPD42Sensor::interrupt() {
}

void PPD42Sensor::_handleInterruptPM10() {
	if (digitalRead(_PPD_PM10_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		_triggerStartMicrosPM10 = micros();
	else	// not LOW, thus end of the trigger
		_triggeredTotalMicrosPM10 += (micros() - _triggerStartMicrosPM10);
}

void PPD42Sensor::_handleInterruptPM25() {
	if (digitalRead(_PPD_PM25_PIN) == LOW)
		// the sensor pulls the pin low to trigger
		_triggerStartMicrosPM25 = micros();
	else	// not LOW, thus end of the trigger
		_triggeredTotalMicrosPM25 += (micros() - _triggerStartMicrosPM25);
}

void PPD42Sensor::process() {
    
}
String PPD42Sensor::report()  {
    if (!_measured)
        read();
    return buildReport(&measuredData);
}

String PPD42Sensor::buildReport(sensorData *sData)  {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    PPD42Data * ppd42Data = reinterpret_cast <PPD42Data*>(sData);
    root["Sensor"] = "PPD42";
    root["PM2.5"] = ppd42Data->PM25Ppm;
    root["PM1.0"] = ppd42Data->PM10Ppm;
    root.printTo(response,sizeof(response));
    return response;
}
