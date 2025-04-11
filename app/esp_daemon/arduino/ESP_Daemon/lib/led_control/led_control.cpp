#include "led_control.h"
#include "config.h"
#include <Arduino.h>

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// use volatile to ensure the variable is updated correctly in ISR
volatile int mode = 0;
volatile int sensor_mode = 0;
unsigned long last_override_time = 0;
const unsigned long override_duration = LED_OVR_DURATION;
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
  static bool wipeOn = true;

  if (millis() - lastUpdate >= wait) {
    strip.setBrightness(LED_BRIGHTNESS); // Reset to default brightness
    if (wipeOn) {
      strip.setPixelColor(pixelIndex, color);
    } else {
      strip.setPixelColor(pixelIndex, 0);
    }
    strip.show();
    pixelIndex++;

    if (pixelIndex >= strip.numPixels()) {
      pixelIndex = 0;
      wipeOn = !wipeOn;
    }
    lastUpdate = millis();
  }
}

void breathingEffectNonBlocking(uint32_t color, int cycles) {
  static int brightness = 0;
  static int direction = 5; // Increased step size for faster transition
  static int cycleCount = 0;

  brightness += direction;

  if (brightness >= 255 || brightness <= 0) {
    direction = -direction;
    if (brightness <= 0) {
      cycleCount++;
      if (cycleCount >= cycles) {
        cycleCount = 0;
        brightness = 0;
        direction = 5;
        return;
      }
    }
  }

  strip.setBrightness(brightness);
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void rainbowNonBlocking(int wait) {
  static long firstPixelHue = 0;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate >= wait) {
    strip.setBrightness(LED_BRIGHTNESS); // Reset to default brightness
    strip.rainbow(firstPixelHue);
    strip.show();
    firstPixelHue += 256;
    if (firstPixelHue >= 5 * 65536) {
      firstPixelHue = 0;
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
