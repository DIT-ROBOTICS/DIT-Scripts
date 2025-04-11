#include "led_control.h"
#include "config.h"
#include <Arduino.h>

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// use volatile to ensure the variable is updated correctly in ISR
volatile int mode = 0;
volatile int sensor_mode = 0;
unsigned long last_override_time = 0;
const unsigned long override_duration = 3000;
int current_mode;

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

void breathingEffect(uint32_t color, int cycles) {
  for (int cycle = 0; cycle < cycles; cycle++) {
    for (int brightness = 0; brightness <= 250; brightness += 25) {
      strip.setBrightness(brightness);
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, color);
      }
      strip.show();
      delay(5);
    }
    for (int brightness = 250; brightness >= 0; brightness -= 25) {
      strip.setBrightness(brightness);
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, color);
      }
      strip.show();
      delay(5);
    }
  }
  strip.setBrightness(LED_BRIGHTNESS); // Reset to default brightness
}

void rainbow(int wait){
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}

void colorWipeNonBlocking(uint32_t color, int wait) {
  static int pixelIndex = 0;
  static unsigned long lastUpdate = 0;
  static bool wipeOn = true; // Track whether the wipe is turning LEDs on or off

  if (millis() - lastUpdate >= wait) {
    if (wipeOn) {
      strip.setPixelColor(pixelIndex, color); // Turn LED on
    } else {
      strip.setPixelColor(pixelIndex, 0); // Turn LED off
    }
    strip.show();
    pixelIndex++;

    if (pixelIndex >= strip.numPixels()) {
      pixelIndex = 0; // Reset for the next call
      wipeOn = !wipeOn; // Toggle between on and off
    }
    lastUpdate = millis();
  }
}

void breathingEffectNonBlocking(uint32_t color, int cycles) {
  static int brightness = 0;
  static int direction = 20; // Increased step size for faster transition
  static unsigned long lastUpdate = 0;
  static int cycleCount = 0;

  if (millis() - lastUpdate >= 20) { // Reduced timing for faster effect
    brightness += direction;

    if (brightness >= 255 || brightness <= 0) {
      direction = -direction; // Reverse direction
      if (brightness <= 0) {
        cycleCount++;
        if (cycleCount >= cycles) {
          cycleCount = 0; // Reset for the next call
          brightness = 0;
          direction = 20;
          return;
        }
      }
    }

    strip.setBrightness(brightness);
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
    lastUpdate = millis();
  }
}

void rainbowNonBlocking(int wait) {
  static long firstPixelHue = 0;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate >= wait) {
    strip.rainbow(firstPixelHue);
    strip.show();
    firstPixelHue += 256;
    if (firstPixelHue >= 5 * 65536) {
      firstPixelHue = 0; // Reset for the next call
    }
    lastUpdate = millis();
  }
}

void LEDTask(void* pvParameters) {
  for (;;) {
    if (mode == SIMA_CMD || mode == EME_ENABLE || mode == EME_DISABLE) {
      if (millis() - last_override_time > override_duration) {
        mode = -1;
      }
    }

    int current_mode = (mode == -1) ? sensor_mode : mode;

    switch (current_mode) {
      case SIMA_CMD:
        breathingEffectNonBlocking(strip.Color(255, 255, 0), 5);
        break;
      case EME_DISABLE:
        breathingEffectNonBlocking(strip.Color(255, 0, 0), 5);
        break;
      case EME_ENABLE:
        breathingEffectNonBlocking(strip.Color(0, 255, 0), 5);
        break;
      case BATT_DISCONNECTED:
        colorWipeNonBlocking(strip.Color(0, 0, 255), 50);
        break;
      case BATT_LOW:
        colorWipeNonBlocking(strip.Color(255, 0, 0), 50);
        break;
      default:
        rainbowNonBlocking(5); // Adjusted wait for smoother rainbow
    }
    vTaskDelay(1); // Yield to other tasks
  }
}
