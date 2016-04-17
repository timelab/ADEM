// 
// 
// 

#include "gps_SwSerial.h"

#if defined(__AVR__)
#include <util/delay.h>
#endif

#ifdef DEBUG_GPS:
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

SwSerialGPS::SwSerialGPS(int rx, int tx, int bd) {
    swserial = new SoftwareSerial(rx, tx, false, 64);
    tinygps = new TinyGPSPlus();
    baud = bd;
}

SwSerialGPS::SwSerialGPS() {
    SwSerialGPS(4, 0, 9600);
}

void SwSerialGPS::begin(void) {
    swserial->begin(baud);
}

void SwSerialGPS::end() {
}

void SwSerialGPS::write() {
}

void SwSerialGPS::process() {
}

void SwSerialGPS::read() {

    unsigned long start = millis();
    do {
        while (swserial->available() > 0) {
            char c = swserial->read();
            __LOG(c);
            tinygps->encode(c);
        }
    } while (millis() - start < 100);

    date = tinygps->date;
    time = tinygps->time;
    location = tinygps->location;
    satellites = tinygps->satellites;
}

String SwSerialGPS::FormatDateTime(TinyGPSDate date, TinyGPSTime time) {
    char gpsDateTime[26] = "";
    sprintf(gpsDateTime, "%04d-%02d-%02dT%02d:%02d:%02d.%02d0Z", date.year(), date.month(), date.day(), time.hour(), time.minute(), time.second(), time.centisecond());
    return gpsDateTime;
}

String SwSerialGPS::report()  {

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char response[200];
    root["Sensor"] = "GPS";

    if (tinygps->date.isValid() && tinygps->time.isValid()) {
        root["Time"] = FormatDateTime(date, time);
    }

    if (tinygps->location.isValid()) {
        ready = true;
        root["Lattitude"] = location.lat();
        root["Longitude"] = location.lng();
    } else {
        ready = false;
    }

    if (tinygps->satellites.isValid()) {
        root["Satellites"] = satellites.value();
    }

    root.printTo(response, sizeof(response));
    return response;
}
