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
 * Copyright 2016 Dag Wieers, Stijn Diependaele
 *
 */

#include <ESP8266WiFi.h>

extern "C" {
   ADC_MODE(ADC_VCC);
}

#include "battery_lipo.h"

#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

LipoBattery::LipoBattery() {
}

LipoBattery::~LipoBattery() {
}

void LipoBattery::begin() {
  Serial.print("Initializing battery sensor... ");
  Serial.println("OK");
}

void LipoBattery::end () {
}

void LipoBattery::read() {
  vcc = ESP.getVcc();

  __LOG("{\"Sensor\":\"Battery\",\"VCC\":"); __LOG(vcc);
  __LOG(",\"Voltage\":"); __LOG((float) vcc / 1024.0); __LOGLN("}");
  if (vcc < 2900) {
    empty = true;
    __LOGLN("Battery is empty.");
  } else if (vcc < 3000) {
    low = true;
    __LOGLN("Battery is low.");
  } else if (vcc >= 4000) {
    full =true;
    __LOGLN("Battery is full.");
  } else {
    empty = false;
    low = false;
    full = false;
  }
}

void LipoBattery::write() {
}

void LipoBattery::process() {
}

String LipoBattery::report()  {
//    return "Battery at " + vcc;
}
