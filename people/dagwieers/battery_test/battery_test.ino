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

// NOTE: Ensure that nothing is connected to ADC (A0) pin !

#include <battery_lipo.h>

#define SERIAL_BAUD 74880

LipoBattery battery = LipoBattery();

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Booting...");

    battery.begin();
}

void loop() {
  battery.read();
//  Serial.print("Battery currently at ");
//  Serial.print((float)battery.vcc / 1024.0);
//  Serial.println("V.");
  delay(1000);
}
