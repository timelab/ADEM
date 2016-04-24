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

#include "led_NeoPixel.h"

NeoPixelLed::NeoPixelLed(uint16_t n, uint8_t p, neoPixelType t) {
    neopixel = Adafruit_NeoPixel(n, p, t);
}

NeoPixelLed::NeoPixelLed() {
    NeoPixelLed(1, 5, NEO_GRB + NEO_KHZ800);
}

NeoPixelLed::~NeoPixelLed() {
    delete &neopixel;
}

void NeoPixelLed::begin() {
    Serial.print("Initializing LED... ");
    neopixel.begin();
    // FIXME: Brightness does not seem to work, so we correct using colors
    neopixel.setBrightness(31);
    neopixel.show();
    Serial.println("OK");
}

void NeoPixelLed::end () {
}

void NeoPixelLed::read() {
}

void NeoPixelLed::write() {
}

void NeoPixelLed::process() {
}

String NeoPixelLed::report()  {
    return "Nothing to report.";
}

void NeoPixelLed::setbrightness(uint8_t br) {
    neopixel.setBrightness(br);
    neopixel.show();
}

void NeoPixelLed::setcolor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
    neopixel.setPixelColor(n, r, g, b);
    neopixel.show();
}

void NeoPixelLed::setcolor(uint16_t n, uint32_t c) {
    neopixel.setPixelColor(n, c);
    neopixel.show();
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t NeoPixelLed::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
