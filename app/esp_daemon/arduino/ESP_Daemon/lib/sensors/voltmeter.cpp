#include "voltmeter.h"
#include "config.h"
#include <Arduino_JSON.h>
#include <ESPAsyncWebServer.h>

extern AsyncEventSource events;

float Vbattf = 0.0;
uint32_t Vbatt = 0;
float offset = VOLTMETER_OFFSET;
volatile int interruptCounter = 0;
hw_timer_t* _timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void initVoltmeter() {
  pinMode(VOLTMETER_PIN, INPUT);
  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, &onTimer, true);
  timerAlarmWrite(_timer, TIMER_PERIOD_US, true);
  timerAlarmEnable(_timer);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void voltmeter() {
  Vbatt = 0;
  for (int i = 0; i < SLIDING_WINDOW_SIZE; i++) {
    Vbatt += analogReadMilliVolts(VOLTMETER_PIN);
  }
  Vbattf = VOLTMETER_CALIBRATION * Vbatt / SLIDING_WINDOW_SIZE / 1000.0 + offset;
  if (Vbattf < 3) Vbattf = 0.0;
}

String getSensorReadings() {
  JSONVar readings;
  readings["sensor"] = String(Vbattf);
  readings["GND"] = 0;

  if (Vbattf < 3) {
    readings["batteryStatus"] = "battery_disconnect";
  } else if (Vbattf < 17.5) {
    readings["batteryStatus"] = "low_battery";
  } else {
    readings["batteryStatus"] = "normal";
  }

  return JSON.stringify(readings);
}

void voltmeterTask(void* pvParameters) {
  for (;;) {
    voltmeter();

    if (interruptCounter > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter--;
      portEXIT_CRITICAL(&timerMux);

      events.send(getSensorReadings().c_str(), "new_readings", millis());
    }

    delay(100);
  }
}
