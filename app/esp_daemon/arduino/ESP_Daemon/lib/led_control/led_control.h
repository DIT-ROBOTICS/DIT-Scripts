#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
extern volatile int mode;

void initLED();
void colorWipe(uint32_t color, int wait);
void rainbow(int wait);
void LEDTask(void* pvParameters);

#endif
