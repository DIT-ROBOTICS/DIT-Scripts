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

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <AsyncTCP.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <Adafruit_NeoPixel.h>

// Micro-ROS 
#include <micro_ros_platformio.h>

// Core ROS2 libraries for creating nodes, publishers, and executors
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Standard ROS2 message types
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// Ensure that the transport layer being used is Arduino Serial.
// If it's not, compilation is stopped and error is printed.
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Define ROS2 objects for a publisher, a message, an executor, support objects, an allocator, a node, and a timer
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t float_publisher;
std_msgs__msg__Float32 float_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Infinite error loop function. If something fails, the device will get stuck here
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

float Vbattf = 0.0;
// This is the function that will be called every time the timer expires
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish integer message
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // Publish float message
    float_msg.data = Vbattf;
    RCSOFTCHECK(rcl_publish(&float_publisher, &float_msg, NULL));
    msg.data++;
    float_msg.data = Vbattf;
  }
}

void initROS(void) {
  // Configure Micro-ROS library to use Arduino serial
  set_microros_serial_transports(Serial);
  // Allow some time for everything to start properly
  delay(2000);

  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Initialize a ROS publisher with the name "micro_ros_platformio_node_publisher" to publish Int32 messages
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // Initialize a ROS publisher for Float32 messages
  RCCHECK(rclc_publisher_init_default(
    &float_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "battery_voltage"));

  // Initialize a timer with a period of 1 second which calls the function timer_callback() every time it expires
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Add our timer to the executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize our message data to 0
  msg.data = 0;

  // Initialize our float message data to 0.0
  float_msg.data = 0.0;
}

#define LED_PIN   D1
#define LED_COUNT 30
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int mode = 0;

WiFiManager wifiManager;

TaskHandle_t Task1;
TaskHandle_t Task2;

volatile int interruptCounter;
hw_timer_t *_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

uint32_t Vbatt = 0;
// float Vbattf = 0.0;
float offset = 0.65;

// Get Sensor Readings and return JSON object
String getSensorReadings() {
  readings["sensor"] = String(Vbattf);
  readings["GND"] = 0;

  // 17.5V-[LOW]  20.5V-[FULL]
  if (Vbattf < 3) {
    readings["batteryStatus"] = "battery_disconnect";
    mode = 2;
  } else if (Vbattf < 17.5) {
    readings["batteryStatus"] = "low_battery";
    mode = 1;
  } else {
    readings["batteryStatus"] = "normal";
    mode = 0;
  }

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {
  wifiManager.autoConnect("DIT-2025-00-ESP");
  if (!MDNS.begin("dit-2025-00-esp")) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println(WiFi.localIP());
}

void voltmeter() {
  Vbatt = 0;
  for (int i = 0; i < 64; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A0);  // ADC with correction
  }
  Vbattf = 7.81 * Vbatt / 64 / 1000.0 + offset;  // R1 = 1.5M ohm, R2 = 220k ohm
  if (Vbattf < 3) Vbattf = 0.00;
  // Serial.print("batteryVoltage:");
  // Serial.println(Vbattf, 1);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                //  Update strip to match
    delay(wait);                                 //  Pause for a moment
  }
}

void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) {   // Repeat 10 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color);  // Set pixel 'c' to value 'color'
      }
      strip.show();  // Update strip with new contents
      delay(wait);   // Pause for a moment
    }
  }
}

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show();  // Update strip with new contents
    delay(wait);   // Pause for a moment
  }
}

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;           // First pixel starts at red (hue 0)
  for (int a = 0; a < 30; a++) {   // Repeat 30 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue));  // hue -> RGB
        strip.setPixelColor(c, color);                        // Set pixel 'c' to value 'color'
      }
      strip.show();                 // Update strip with new contents
      delay(wait);                  // Pause for a moment
      firstPixelHue += 65536 / 90;  // One cycle of color wheel over 90 frames
    }
  }
}

void Task1code(void *pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // Execute pending tasks in the executor. This will handle all ROS communications.
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    voltmeter();
    if (interruptCounter > 0) {

      portENTER_CRITICAL(&timerMux);
      interruptCounter--;
      portEXIT_CRITICAL(&timerMux);

      events.send(getSensorReadings().c_str(), "new_readings", millis());
    }
  }
}

void Task2code(void *pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    switch (mode) 
    {
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
    // theaterChase(strip.Color(255, 0, 0), 50);
    // theaterChaseRainbow(50);
  }
}

void setup() {
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);

  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(128); // Set BRIGHTNESS

  _timer = timerBegin(0, 80, true);
  timerAttachInterrupt(_timer, &onTimer, true);
  timerAlarmWrite(_timer, 1000000, true);
  timerAlarmEnable(_timer);

  pinMode(A0, INPUT);  // ADC

  // Serial port for debugging purposes
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
  initROS();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();
  });

  events.onConnect([](AsyncEventSourceClient *client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("Test", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  // Start server
  AsyncElegantOTA.begin(&server);  // Start ElegantOTA
  server.begin();
}

void loop() {}
