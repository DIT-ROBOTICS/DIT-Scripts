/******************************
  Requirements:
    Adafruit_NeoPixel
    Arduino_JSON
    AsyncElegantOTA-2.2.8
    AsyncTCP
    ESP_Async_WebServer
    WiFiManager
  [ esp32 v2.0.17 ]
******************************/

// ========== Includes ==========
// Arduino/ESP32 core
#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>

// Network and Web
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Arduino_JSON.h>

// LED
#include <Adafruit_NeoPixel.h>

// ESP-NOW
#include <esp_now.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

// ========== Defines ==========
#define LED_PIN       D1
#define LED_COUNT     30
#define TIMER_PERIOD_US 1000000

#define RCCHECK(fn)        { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn)    { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// ========== Global Variables ==========

// LED
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// WiFi / Server
WiFiManager wifiManager;
AsyncWebServer server(80);
AsyncEventSource events("/events");

// ESP-NOW
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = { 0x94, 0xa9, 0x90, 0x0b, 0x07, 0x00 };
struct struct_message {
  int sima_start;
} myData;

// micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t float_publisher;
std_msgs__msg__Float32 float_msg;

rcl_subscription_t int_subscriber;
std_msgs__msg__Int32 int_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Tasks and Timer
TaskHandle_t Task1;
TaskHandle_t Task2;
volatile int interruptCounter = 0;
hw_timer_t* _timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Others
volatile int mode = 0;
float offset = 0.65;
uint32_t Vbatt = 0;
float Vbattf = 0.0;

// ========== Function Prototypes ==========
void error_loop();
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void sima_callback(const void* msgin);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

void initROS();
void initSPIFFS();
void initWiFi();
void initESPNow();

void voltmeter();
void IRAM_ATTR onTimer();

void colorWipe(uint32_t color, int wait);
void rainbow(int wait);

void Task1code(void* pvParameters);
void Task2code(void* pvParameters);

String getSensorReadings();

// ========== Core Functions ==========

void error_loop() {
  while (1) delay(100);
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCSOFTCHECK(rcl_publish(&float_publisher, &float_msg, NULL));
    msg.data++;
    float_msg.data = Vbattf;
  }
}

void sima_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  mode = msg->data;
  myData.sima_start = mode;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
  Serial.println(result == ESP_OK ? "ESP-NOW Success" : "ESP-NOW Error");
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ========== Initialization Functions ==========

void initROS() {
  set_microros_serial_transports(Serial);
  delay(2000);
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 100);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, "esp32_watchdog", "", &support));
  RCCHECK(rclc_publisher_init_default(&publisher, &node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32_counter"));
  RCCHECK(rclc_publisher_init_default(&float_publisher, &node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/robot_status/battery_voltage"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_subscription_init_default(&int_subscriber, &node,
           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/sima/start"));
  RCCHECK(rclc_executor_add_subscription(&executor, &int_subscriber, &int_msg, &sima_callback, ON_NEW_DATA));

  msg.data = 0;
  float_msg.data = 0.0;
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS mount failed");
  } else {
    Serial.println("SPIFFS mounted");
  }
}

void initWiFi() {
  wifiManager.autoConnect("DIT-2025-00-ESP");
  if (!MDNS.begin("dit-2025-00-esp")) {
    Serial.println("mDNS init failed");
    return;
  }
  Serial.println(WiFi.localIP());
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW peer add failed");
    return;
  }
}

// ========== Sensor/LED Utility ==========

void voltmeter() {
  Vbatt = 0;
  for (int i = 0; i < 64; i++) {
    Vbatt += analogReadMilliVolts(A0);
  }
  Vbattf = 7.81 * Vbatt / 64 / 1000.0 + offset;
  if (Vbattf < 3) Vbattf = 0.0;
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
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

// ========== Tasks ==========

void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

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

void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000)));
    voltmeter();

    if (interruptCounter > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter--;
      portEXIT_CRITICAL(&timerMux);

      events.send(getSensorReadings().c_str(), "new_readings", millis());
    }
  }
}

// ========== Setup and Loop ==========

void setup() {
  Serial.begin(115200);

  // Start micro-ROS initialization first
  initROS();

  // Start tasks
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);

  // Initialize LED strip
  strip.begin();
  strip.show();
  strip.setBrightness(128);

  // Initialize timer for voltmeter
  pinMode(A0, INPUT);
  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, &onTimer, true);
  timerAlarmWrite(_timer, TIMER_PERIOD_US, true);
  timerAlarmEnable(_timer);

  // Attempt to initialize WiFi and related services
  initWiFi();
  initSPIFFS();
  initESPNow();

      server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send(SPIFFS, "/index.html", "text/html");
    });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/readings", HTTP_GET, [](AsyncWebServerRequest* request) {
      String json = getSensorReadings();
      request->send(200, "application/json", json);
    });

    events.onConnect([](AsyncEventSourceClient* client) {
      if (client->lastId()) {
        Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
      }
      client->send("Test", NULL, millis(), 10000);
    });

    server.addHandler(&events);
    AsyncElegantOTA.begin(&server);
    server.begin();
  }

void loop() {
  // Empty loop; tasks run independently
}
