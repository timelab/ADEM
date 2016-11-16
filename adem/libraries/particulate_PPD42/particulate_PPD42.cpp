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

  XXX: This device does not measure PM10, but PM1 !!
*/

#include <Arduino.h>
#include <particulate_PPD42.h>

/*
    instantiate and initialize the static variables that are used in
    the static interrupt functions that handle the hardware interrupts
    triggered by the PPD42 dust sensor
*/

int PPD42Sensor::_PPD_PM1_PIN = 12;
int PPD42Sensor::_PPD_PM25_PIN = 13;
volatile uint32_t PPD42Sensor::_triggerStartMicrosPM1 = 0;
volatile uint32_t PPD42Sensor::_triggerStartMicrosPM25 = 0;
volatile uint32_t PPD42Sensor::_triggeredTotalMicrosPM1 = 0;
volatile uint32_t PPD42Sensor::_triggeredTotalMicrosPM25 = 0;
#ifdef DEBUG_PPD42
volatile uint32_t PPD42Sensor::_totalInterruptsPM1;
volatile uint32_t PPD42Sensor::_totalInterruptsPM25;
#endif

PPD42Sensor::PPD42Sensor(int PM1_PIN, int PM25_PIN){
    _PPD_PM1_PIN = PM1_PIN;
    _PPD_PM25_PIN = PM25_PIN;
    measuredData.ID = PARTICULATE_PPD42;
}

void PPD42Sensor::begin() {
    Serial.print("Initializing particulate sensor... ");
    _measured = false;

    // initialization for the dust sensor
    _readMillisPM1 = millis(); //Fetch the current time
    _readMillisPM25 = millis(); //Fetch the current time
    _triggeredTotalMicrosPM1 = 0;
    _triggeredTotalMicrosPM25 = 0;
#ifdef DEBUG_PPD42
    _totalInterruptsPM1 = 0;
    _totalInterruptsPM25 = 0;
#endif
    pinMode(_PPD_PM1_PIN, INPUT);
    pinMode(_PPD_PM25_PIN, INPUT);
    _activated = true;

    // Attach interrupts handlers to hardware pins
    attachInterrupt(digitalPinToInterrupt(_PPD_PM1_PIN), _handleInterruptPM1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_PPD_PM25_PIN), _handleInterruptPM25, CHANGE);

    Serial.println("OK");
}

void PPD42Sensor::end() {
    _activated = false;
    // detach the interrupts
    detachInterrupt(_PPD_PM1_PIN);
    detachInterrupt(_PPD_PM25_PIN);
}

void PPD42Sensor::read() {
    measuredData.PM1Ppm = readPM1Ppm();
    measuredData.PM25Ppm = readPM25Ppm();
    _measured = true;
}

uint32_t PPD42Sensor::readPM1Ppm() {
    if (_activated) {
        uint32_t _currentMillis = millis();
        uint32_t _sampledMillis = _currentMillis - _readMillisPM1;
        uint32_t _triggeredTotalMicros = _triggeredTotalMicrosPM1;
        _readMillisPM1 = _currentMillis;
#ifdef DEBUG_PPD42
        measuredData.triggeredTotalMicrosPM1 = _triggeredTotalMicrosPM1;
        measuredData.sampledMillisPM1 = _sampledMillis;
        measuredData.totalInterruptsPM1 = _totalInterruptsPM1;
        _totalInterruptsPM1 = 0;
#endif
        _triggeredTotalMicrosPM1 = 0;
        // We avoid floating point arithmetic and accept the inaccuracy
        return 1000 * (uint64_t) _triggeredTotalMicros / _sampledMillis;
//        return 1000.0 * _triggeredTotalMicros / _sampledMillis;
    }
    else
        return 0;
}

uint32_t PPD42Sensor::readPM25Ppm() {
    if (_activated) {
        uint32_t _currentMillis = millis();
        uint32_t _sampledMillis = _currentMillis - _readMillisPM25;
        uint32_t _triggeredTotalMicros = _triggeredTotalMicrosPM25;
        _readMillisPM25 = _currentMillis;
#ifdef DEBUG_PPD42
        measuredData.triggeredTotalMicrosPM25 = _triggeredTotalMicrosPM25;
        measuredData.sampledMillisPM25 = _sampledMillis;
        measuredData.totalInterruptsPM25 = _totalInterruptsPM25;
        _totalInterruptsPM25 = 0;
#endif
        _triggeredTotalMicrosPM25 = 0;
        // We avoid floating point arithmetic and accept the inaccuracy
        return 1000 * (uint64_t) _triggeredTotalMicros / _sampledMillis;
//        return 1000.0 * _triggeredTotalMicros / _sampledMillis;
    }
    else
        return 0;
}

void PPD42Sensor::write() {
}

void PPD42Sensor::interrupt() {
}

void PPD42Sensor::_handleInterruptPM1() {
    if (digitalRead(_PPD_PM1_PIN) == LOW)
        // the sensor pulls the pin low to trigger
        _triggerStartMicrosPM1 = micros();
    else    // not LOW, thus end of the trigger
        _triggeredTotalMicrosPM1 += (micros() - _triggerStartMicrosPM1);
#ifdef DEBUG_PPD42
    _totalInterruptsPM1++;
#endif
}

void PPD42Sensor::_handleInterruptPM25() {
    if (digitalRead(_PPD_PM25_PIN) == LOW)
        // the sensor pulls the pin low to trigger
        _triggerStartMicrosPM25 = micros();
    else    // not LOW, thus end of the trigger
        _triggeredTotalMicrosPM25 += (micros() - _triggerStartMicrosPM25);
#ifdef DEBUG_PPD42
    _totalInterruptsPM25++;
#endif
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
    root["PM1"] = ppd42Data->PM1Ppm;
    root["PM2.5"] = ppd42Data->PM25Ppm;
#ifdef DEBUG_PPD42
    root["PM1ints"] = ppd42Data->totalInterruptsPM1;
    root["PM2.5ints"] = ppd42Data->totalInterruptsPM25;
    root["PM1totMic"] = ppd42Data->triggeredTotalMicrosPM1;
    root["PM2.5totMic"] = ppd42Data->triggeredTotalMicrosPM25;
    root["PM1samMil"] = ppd42Data->sampledMillisPM1;
    root["PM2.5samMil"] = ppd42Data->sampledMillisPM25;
#endif
    root.printTo(response,sizeof(response));
    return response;
}
