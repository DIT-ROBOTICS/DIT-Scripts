#include <Arduino.h>
#include "config.h"
#include "led_control.h"
#include "voltmeter.h"
#include "espnow_comm.h"
#include "ros_node.h"
#include "web_server.h"
#include "wifi_config.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

void microROSTask(void* pvParameters) {
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_TIMER_MS));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  // ================================================================
  //  NOTE: DO NOT CHANGE THE ORDER OF THE FOLLOWING INITIALIZATIONS
  // ================================================================

  initROS();

  initLED();
  initVoltmeter();

  xTaskCreatePinnedToCore(LEDTask, "LED Task", 10000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(voltmeterTask, "Sensor Task", 10000, NULL, 1, &Task2, 1);
  xTaskCreatePinnedToCore(microROSTask, "microROS", 10000, NULL, 1, NULL, 1);

  initESPNow();
  
  initSPIFFS();
  initWiFi();
  initWebServer();
}

void loop() {}
