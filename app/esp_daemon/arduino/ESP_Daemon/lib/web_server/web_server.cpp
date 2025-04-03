#include "web_server.h"
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Arduino_JSON.h>
#include <AsyncElegantOTA.h>

AsyncWebServer server(80);
AsyncEventSource events("/events");

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed");
  }
}

void initWebServer() {
  // [ 2025-04-04 ]
  // |   Removed the initSPIFFS() call here as it is already invoked in main.cpp. 
  // |   This ensures SPIFFS is initialized before starting the web server and WiFi manager.
  // initSPIFFS();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{}");
  });

  events.onConnect([](AsyncEventSourceClient* client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
    }
    client->send("connected", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  AsyncElegantOTA.begin(&server);
  server.begin();
}
