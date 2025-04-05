#ifndef CONFIG_H
#define CONFIG_H

// WiFi and mDNS
#define HOSTNAME        "DIT-2025-00-ESP"
#define MDNS_NAME       "dit-2025-00-esp"

// ESP-NOW for SIMA communication
#define BROADCAST_ADDR { 0x94, 0xa9, 0x90, 0x0b, 0x07, 0x00 }

// micro-ROS
#define ROS_NODE_NAME       "esp_daemon"
#define ROS_DOMAIN_ID       100
#define ROS_TIMER_MS        250
#define MROS_TIMEOUT_MS     100
#define MROS_PING_INTERVAL  1000
#define RELAY_PIN           D2

// RGB LED strip 
#define LED_PIN         D1
#define LED_COUNT       30
#define LED_BRIGHTNESS  128

// Voltmeter - Battery voltage measurement
// | Formula:
// |    Vbattf = (VOLTMETER_CALIBRATION * Vbatt / SLIDING_WINDOW_SIZE / 1000.0) + VOLTMETER_OFFSET;
// | Note:
// |    (A0 == D0) on Xiao ESP32C3
#define VOLTMETER_PIN           A0
#define VOLTMETER_CALIBRATION   7.81
#define VOLTMETER_OFFSET        0.65
#define SLIDING_WINDOW_SIZE     64
#define TIMER_PERIOD_US         1000000
// constexpr uint32_t TIMER_PERIOD_US = 1000000;

#endif
