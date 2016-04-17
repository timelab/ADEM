#include <gps_SwSerial.h>
#include <led_NeoPixel.h>

#define DEBUG_GPS 1

#define SERIAL_BAUD 74880

SwSerialGPS gps = SwSerialGPS(4, 0, 9600);

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Booting...");

    Serial.println("LED initialized.");

    gps.begin();
    Serial.println("GPS initialized.");
    delay(500);
}

void loop() {
    gps.read();
    Serial.println(gps.report());

    delay(500);
}