#ifndef CONFIG_H
#define CONFIG_H

// WiFi and mDNS
#define HOSTNAME    "DIT-2025-00-ESP"
#define MDNS_NAME   "dit-2025-00-esp"

// ESP-NOW for SIMA communication
//  [94:a9:90:07:00:78]---[01]
//  [94:a9:90:06:E6:00]---[02]
//  [94:a9:90:0b:86:d8]---[03]
//  [94:a9:90:05:57:d8]---[04]
#define SIMA_01 { 0x94, 0xa9, 0x90, 0x07, 0x00, 0x78 }
#define SIMA_02 { 0x94, 0xa9, 0x90, 0x06, 0xe6, 0x00 }
#define SIMA_03 { 0x94, 0xa9, 0x90, 0x0b, 0x86, 0xd8 }
#define SIMA_04 { 0x94, 0xa9, 0x90, 0x05, 0x57, 0xd8 }

// micro-ROS
#define ROS_NODE_NAME       "esp_daemon"
#define ROS_DOMAIN_ID       00
#define ROS_TIMER_MS        100
#define MROS_TIMEOUT_MS     100
#define MROS_PING_INTERVAL  1000

// Emergency Button
#define RELAY_PIN           D2
#define RELAY_ACTIVE_STATE  LOW
#define RELAY_INITIAL_STATE LOW
#define ENABLE              RELAY_ACTIVE_STATE
#define DISABLE           (!RELAY_ACTIVE_STATE)

// RGB LED strip 
#define LED_PIN             D1
#define LED_COUNT           40
#define LED_BRIGHTNESS      128
#define LED_OVR_DURATION    1000

// Voltmeter - Battery voltage measurement
// | Formula:
// |    Vbattf = (VOLTMETER_CALIBRATION * Vbatt / SLIDING_WINDOW_SIZE / 1000.0) + VOLTMETER_OFFSET;
// |    [ R1 = 1.5M ohm, R2 = 220k ohm ] VC = 7.81 OFFSET = 0.65
// |    [ R1 = 1.5M ohm, R2 = 200k ohm ] VC = 8.50 OFFSET = 0.65
// | Note:
// |    (A0 == D0) on Xiao ESP32C3
#define VOLTMETER_PIN           A0
#define VOLTMETER_CALIBRATION   8.5
#define VOLTMETER_OFFSET        0.1
#define SLIDING_WINDOW_SIZE     64
#define TIMER_PERIOD_US         1000000
// constexpr uint32_t TIMER_PERIOD_US = 1000000;

#endif
