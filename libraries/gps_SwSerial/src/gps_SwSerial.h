// SwSerialGPS.h

#ifndef _SwSerialGPS_h
#define _SwSerialGPS_h
#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif

#define GPS_RX_PIN 4
#define GPS_TX_PIN 0

#define GPS_BAUD 9600

//abstract class Sensor
class SwSerialGPS : public Sensor {
public:
    SwSerialGPS(int rx=4, int tx=0, int bd=9600);
    SwSerialGPS();

    //virtual function must be implemented
    virtual void begin();
    virtual void end();
    virtual void read();
    virtual void write();
    virtual void process();
    virtual String report();

private:
    StaticJsonBuffer<200> jsonBuffer;

    const unsigned char OSS = 0;  // Oversampling Setting

    SoftwareSerial swserial;
    TinyGPSPlus tinygps;

    boolean _error = false;
    boolean readData(void);
    TinyGPSDate date;
    TinyGPSTime time;
    TinyGPSLocation location;
    TinyGPSInteger satellites;
    int baud = 9600;

    float GetData();

    String FormatDateTime(TinyGPSDate date, TinyGPSTime time);
};

#endif
