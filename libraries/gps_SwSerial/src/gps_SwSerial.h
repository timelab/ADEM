// SwSerialGPS.h

#ifndef _SwSerialGPS_h
#define _SwSerialGPS_h
#include <Arduino.h>
#include <Sensor.h> // - Skeleton Library for ADEM sensors.
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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

    boolean ready = false;

private:
    StaticJsonBuffer<200> jsonBuffer;

    TinyGPSDate date;
    TinyGPSTime time;
    TinyGPSLocation location;
    TinyGPSInteger satellites;
    String FormatDateTime(TinyGPSDate date, TinyGPSTime time);

    int baud = 9600;

    SoftwareSerial *swserial;
    TinyGPSPlus *tinygps;

};

#endif
