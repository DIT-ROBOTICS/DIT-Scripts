#include "led_control.h"
#include "config.h"
#include <Arduino.h>

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile int mode = 0;  // use volatile to ensure the variable is updated correctly in ISR

void initLED() {
  strip.begin();
  strip.show();
  strip.setBrightness(LED_BRIGHTNESS);
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void rainbow(int wait){
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}

void LEDTask(void* pvParameters) {
  for (;;) {
    switch (mode) {
      case 2:
        colorWipe(strip.Color(0, 0, 255), 50);
        colorWipe(strip.Color(0, 0, 0), 50);
        break;
      case 1:
        colorWipe(strip.Color(255, 0, 0), 50);
        colorWipe(strip.Color(0, 0, 0), 50);
        break;
      default:
        rainbow(1);
    }
  }
}
