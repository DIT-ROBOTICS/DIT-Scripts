#include "config.h"
#include "led_control.h"
#include "voltmeter.h"
#include "espnow_comm.h"
#include "ros_node.h"
#include "web_server.h"
#include "wifi_config.h"
#include <Arduino.h>
#include <rmw_microros/rmw_microros.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

void microROSTask(void* pvParameters) {
  while (true) {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(MROS_PING_INTERVAL, state = (RMW_RET_OK == rmw_uros_ping_agent(MROS_TIMEOUT_MS, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
          state = WAITING_AGENT;
        }
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(MROS_PING_INTERVAL, state = (RMW_RET_OK == rmw_uros_ping_agent(MROS_TIMEOUT_MS, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_TIMER_MS));
          // vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }
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

  initSPIFFS();
  initWiFi();
  initESPNow();
  initWebServer();
}

void loop() {}
