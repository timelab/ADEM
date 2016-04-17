#include <led_NeoPixel.h>

#define NEOPIXEL_PIN 5

NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    led.begin();
    led.setcolor(0, 63, 0, 0); // Red
}

void loop() {
  led.setcolor(0, 63, 0, 0); // Red
  delay(100);
  led.setcolor(0, 63, 31, 0); // Orange
  delay(100);
  led.setcolor(0, 31, 31, 0); // Yellow
  delay(100);
  led.setcolor(0, 0, 63, 0); // Green
  delay(100);
  led.setcolor(0, 0, 0, 63); // Green
  delay(100);
  led.setcolor(0, 63, 0, 63); // Green
  delay(100);
}
