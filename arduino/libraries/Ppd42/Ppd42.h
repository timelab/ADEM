/*
  Ppd42.h - Library for ADEM PPD42 sensor.
  Created by Lieven Blancke.

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

#include <Arduino.h>
#include <Sensor.h>

class Ppd42 : public Sensor {
  public:
    void begin();
    void end();
    void read();
    void write();
    static void interrupt(); // interrupt functions must be static
    void process();
    Ppd42();
    String report(); // should report a JSON string

    String debug = "";
    unsigned long readPM10Ppm(); // returns the time (in ppm) the sensor triggered
    unsigned long readPM25Ppm();

  private:
    boolean activated = false;
    unsigned long readMillisPM10;
    unsigned long readMillisPM25;

    // Static methods and variables for the interrupt functions
    static void handleInterruptPM10();
    static int PPD_PM10_PIN;
    static volatile boolean triggerStartedPM10;
    static volatile unsigned long triggerStartMicrosPM10;
    static volatile unsigned long triggeredTotalMicrosPM10;
    static void handleInterruptPM25();
    static int PPD_PM25_PIN;
    static volatile boolean triggerStartedPM25;
    static volatile unsigned long triggerStartMicrosPM25;
    static volatile unsigned long triggeredTotalMicrosPM25;
};

#endif
