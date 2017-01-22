/*
 * This file is part of the ADEM project.
 *
 * ADEM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, 
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
 * Copyright 2016 Dag Wieers, Lieven Blancke, Koen Verstringe
 *
 */

#include <gps_I2C.h>

#define GPS_TX_PIN 16
#define GPS_RX_PIN 4
#define GPS_BAUD 9600
#define SERIAL_BAUD 74880

#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif


I2CGps gps = I2CGps(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

void setup() {
  Wire.begin();
  Wire.setClock(100000);

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println("Serial communication... OK");

  __LOGLN();
  __LOGLN("Setup started in DEBUG mode.");

  gps.begin();
}

void loop() {
  gps.read();
  //buffer.write((char *)gps.dataToBuffer(), gps.dataBufferSize());
  __LOGLN(gps.report());
  delay (1000);
}

// vim:syntax=cpp
