// 
// 
// 

#include "gps_SwSerial.h"

#if defined(__AVR__)
#include <util/delay.h>
#endif

SwSerialGPS::SwSerialGPS(int rx, int tx, int bd) {
    swserial = SoftwareSerial(rx, tx, false, 64);
    tinygps = TinyGPSPlus();
    baud = bd;
}

SwSerialGPS::SwSerialGPS() {
    SwSerialGPS(4, 0, 9600);
}

void SwSerialGPS::begin(void) {
    swserial.begin(baud);
}

void SwSerialGPS::end() {
}

void SwSerialGPS::write() {
}

void SwSerialGPS::process() {
}

float SwSerialGPS::GetData(void) {

    unsigned long start = millis();
    do {
        while (swserial.available() > 0) {
            tinygps.encode(swserial.read());
        }
        delay(5);
    } while (millis() - start < 100);

    date = tinygps.date;
    time = tinygps.time;
    location = tinygps.location;
    satellites = tinygps.satellites;
}


void SwSerialGPS::read() {
    GetData();
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

    if (tinygps.date.isValid() && tinygps.time.isValid()) {
        root["Time"] = FormatDateTime(date, time);
    }

    if (tinygps.location.isValid()) {
        root["Lattitude"] = location.lat();
        root["Longitude"] = location.lng();
    }

//    if (tinygps.satellites.isValid()) {
//        root["Satellites"] = satellites.value;
//    }

    root.prettyPrintTo(response, sizeof(response));
    return response;
}
