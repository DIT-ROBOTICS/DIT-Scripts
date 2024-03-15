#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>

// Replace with your network credentials
const char *ssid = "DIT_ROBOTICS";
const char *password = "ditrobotics";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 3000;

uint32_t Vbatt = 0;
float Vbattf = 0.0;
float offset = 0.65;

// Get Sensor Readings and return JSON object
String getSensorReadings() {
  readings["sensor"] = String(Vbattf);
  readings["GND"] = 0;

  // 17.5V-[LOW]  20.5V-[FULL]
  if (Vbattf < 3) {
    readings["batteryStatus"] = "battery_disconnect";
  } else if (Vbattf < 17.5) {
    readings["batteryStatus"] = "low_battery";
  } else {
    readings["batteryStatus"] = "normal";
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
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  if (!MDNS.begin("dit-2024-11-esp")) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(A0, INPUT);  // ADC

  // Serial port for debugging purposes
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();

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

void loop() {
  Vbatt = 0;
  for (int i = 0; i < 64; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A0);  // ADC with correction
  }
  Vbattf = 7.81 * Vbatt / 64 / 1000.0 + offset;  // R1 = 1.5M ohm, R2 = 220k ohm
  if (Vbattf < 3) Vbattf = 0.00;
  Serial.print("batteryVoltage:");
  Serial.println(Vbattf, 1);

  if ((millis() - lastTime) > timerDelay) {
    events.send("ping", NULL, millis());
    events.send(getSensorReadings().c_str(), "new_readings", millis());
    lastTime = millis();
  }
}