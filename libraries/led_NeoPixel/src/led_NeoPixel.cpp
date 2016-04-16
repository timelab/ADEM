//
//
//

#include "led_NeoPixel.h"
/*
NeoPixelLed.cpp - Skeleton library for ADEM sensors.
Created by Dag Wieers, Stijn Diependaele
*/

NeoPixelLed::NeoPixelLed(uint16_t n, uint8_t p, neoPixelType t) {
    neopixel = Adafruit_NeoPixel(n, p, t);
}

NeoPixelLed::NeoPixelLed() {
    NeoPixelLed(1, 5, NEO_GRB + NEO_KHZ800);
}

void NeoPixelLed::begin() {
    neopixel.begin();
    // FIXME: Brightness does not seem to work, so we correct using colors
    neopixel.setBrightness(31);
    neopixel.show();
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
