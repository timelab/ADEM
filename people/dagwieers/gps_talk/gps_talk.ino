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
 * Copyright 2016 Dag Wieers
 *
 */

#include <SoftwareSerial.h>
#include <stdio.h>
#include <string.h>

#define SERIAL_BAUD 74880
//#define GPS_BAUD 4800
#define GPS_BAUD 9600
//#define GPS_BAUD 19200
//#define GPS_BAUD 38400
//#define GPS_BAUD 57600
//#define GPS_BAUD 115200
#define GPS_RX_PIN 4
#define GPS_TX_PIN 0

int idx = 0;
int baudrates[] = { 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 460800, 921600 };

unsigned long start;
SoftwareSerial swserial = SoftwareSerial(GPS_RX_PIN, GPS_TX_PIN, false, 512);

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println("Serial communication... OK");

    Serial.println("You can now send commands to the GPS module.");
    Serial.print("> ");

    swserial.begin(GPS_BAUD);
}

void loop() {
    if (swserial.available() > 0) {
        Serial.write(swserial.read());
    }

    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r' or c == '\n') {
            Serial.println();
        } else {
            Serial.write(c);
        }
        swserial.print(c);
    }
}