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

#define DEBUG_PPD42 1

#include <particulate_PPD42.h>

#if ARDUINO_ARCH_AVR
#define SERIAL_BAUD 38400
#define PM10_PIN 2
#define PM25_PIN 3
#elif ARDUINO_ARCH_ESP8266
#define SERIAL_BAUD 74880
#define PM10_PIN 12
#define PM25_PIN 13
#else
#define SERIAL_BAUD 38400
#define PM10_PIN 12
#define PM25_PIN 13
#endif

PPD42Sensor particulate(PM10_PIN, PM25_PIN);

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Booting...");

    particulate.begin();
}

void loop() {
    delay(60000);
    particulate.read();
    Serial.println(particulate.report());
}