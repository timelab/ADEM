#include <led_NeoPixel.h>

#define NEOPIXEL_PIN 5

NeoPixelLed led = NeoPixelLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    led.begin();
    led.setcolor(0, 63, 0, 0); // Red
}

void loop() {
    led.setcolor(0, 63, 0, 63); // Purple
}
