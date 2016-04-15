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
    neopixel.setBrightness(63);
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
