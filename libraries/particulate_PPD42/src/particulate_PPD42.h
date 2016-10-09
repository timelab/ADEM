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
  Ppd42.h - Library for ADEM PPD42 sensor.

	You can test this library feeding a square wave with a known
	duty cycle into the PM10 input pin. The readPM10() function
	should return a value around 500000 for a 50% duty cycle
	square wave.
	
	Worked on an ESP8266 at 1% accuracy with duty cycles ranging
	from 20% to 80% between 1Hz and 33kHz. A 50% duty cycle at
	60kHz worked buth the ESP crashed at 70kHz, that's 140000
	interrupts per second!
*/

#ifndef Ppd42_h
#define Ppd42_h

#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <Arduino.h>
struct PPD42Data : sensorData {
    long PM10Ppm;
    long PM25Ppm;
#ifdef DEBUG
    uint32_t triggeredTotalMicrosPM10;
    uint32_t triggeredTotalMicrosPM25;
    uint32_t sampledMillisPM10;
    uint32_t sampledMillisPM25;
    uint32_t totalInterruptsPM10;
    uint32_t totalInterruptsPM25;
#endif
};

class PPD42Sensor : public Sensor {
  public:
    void begin();
    void end();
    void read();
    void write();
    static void interrupt(); // interrupt functions must be static
    void process();
    PPD42Sensor(int, int);
    String report(); // should report a JSON string
    virtual String buildReport(sensorData *sData);

    uint32_t readPM10Ppm(); // returns the time (in ppm) the sensor triggered
    uint32_t readPM25Ppm();

  private:
    boolean _activated = false;
    PPD42Data measuredData;
    static int _PPD_PM10_PIN;
    static int _PPD_PM25_PIN;
    uint32_t _readMillisPM10;
    uint32_t _readMillisPM25;

    // Static methods and variables for the interrupt functions
    static void _handleInterruptPM10();
    static void _handleInterruptPM25();
    static volatile uint32_t _triggerStartMicrosPM10;
    static volatile uint32_t _triggerStartMicrosPM25;
    static volatile uint32_t _triggeredTotalMicrosPM10;
    static volatile uint32_t _triggeredTotalMicrosPM25;
#ifdef DEBUG
    static volatile uint32_t _totalInterruptsPM10;
    static volatile uint32_t _totalInterruptsPM25;
#endif
};

#endif
