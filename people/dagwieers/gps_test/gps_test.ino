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

#include <gps_SwSerial.h>

#define DEBUG_GPS 1

#define SERIAL_BAUD 74880

SwSerialGPS gps = SwSerialGPS(4, 0, 9600);

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Booting...");

    gps.begin();
    Serial.println("GPS initialized.");
    delay(500);
}

void loop() {
    gps.read();
    Serial.println(gps.report());

    delay(500);
}