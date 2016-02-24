/*
  Ppd42.h - Library for ADEM PPD42 sensor.
  Created by Lieven Blancke.
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
    double readPM10();
    double readPM25();

  private:
    boolean activated = false;
    unsigned long readTimePM10;
    unsigned long readTimePM25;

    // Static methods and variables for the interrupt functions
    static void handleInterruptPM10();
    static int PPD_PM10_PIN;
    static volatile boolean triggerStartedPM10;
    static volatile unsigned long triggerStartTimePM10;
    static volatile unsigned long triggeredTotalTimePM10;
    static void handleInterruptPM25();
    static int PPD_PM25_PIN;
    static volatile boolean triggerStartedPM25;
    static volatile unsigned long triggerStartTimePM25;
    static volatile unsigned long triggeredTotalTimePM25;
};

#endif
