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

#include <led_NeoPixel.h>

#define NEOPIXEL_PIN 5

NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    led.begin();
    led.setcolor(0, 63, 0, 0); // Red
}

void loop() {
  led.setcolor(0, 63, 0, 0);   // Red
  delay(100);
  led.setcolor(0, 63, 31, 0);  // Orange
  delay(100);
  led.setcolor(0, 31, 31, 0);  // Yellow
  delay(100);
  led.setcolor(0, 0, 63, 0);   // Green
  delay(100);
  led.setcolor(0, 0, 0, 63);   // Blue
  delay(100);
  led.setcolor(0, 63, 0, 63);  // Purple
  delay(100);
}
