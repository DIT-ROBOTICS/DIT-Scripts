#include "web_server.h"
#include "ros_node.h"   // Include for emergency_msg
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Arduino_JSON.h>
#include <AsyncElegantOTA.h>

AsyncWebServer server(80);
AsyncEventSource events("/events");

// For accessing the ROS emergency message
extern std_msgs__msg__Bool emergency_msg;

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
  
  // Serve the main index page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  // Serve static files from SPIFFS
  server.serveStatic("/", SPIFFS, "/");

  // Endpoint for sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{}");
  });

  // Emergency button endpoint - handles both form data and JSON requests
  server.on("/emergency", HTTP_POST, 
    [](AsyncWebServerRequest* request) {
      String response = "{\"success\":false,\"message\":\"Invalid request\"}";
      
      // Process form data submission
      if (request->hasParam("emergency", true)) {
        String emergencyValue = request->getParam("emergency", true)->value();
        
        // Set emergency state based on parameter value
        if (emergencyValue == "true" || emergencyValue == "1") {
          emergency_msg.data = true;
          emergency_callback(&emergency_msg);
          response = "{\"success\":true,\"message\":\"Emergency mode ENABLED\"}";
          // Serial.println("Emergency mode ENABLED via web interface");
        } else if (emergencyValue == "false" || emergencyValue == "0") {
          emergency_msg.data = false;
          emergency_callback(&emergency_msg);
          response = "{\"success\":true,\"message\":\"Emergency mode DISABLED\"}";
          // Serial.println("Emergency mode DISABLED via web interface");
        }
      }
      
      // Process JSON submission (stored by body handler)
      if (request->_tempObject != nullptr) {
        String jsonStr = (const char*)request->_tempObject;
        JSONVar jsonObj = JSON.parse(jsonStr);
        
        if (JSON.typeof(jsonObj) == "object" && jsonObj.hasOwnProperty("emergency")) {
          bool emergencyState = (bool)jsonObj["emergency"];
          emergency_msg.data = emergencyState;
          
          // Call emergency callback to ensure immediate action
          emergency_callback(&emergency_msg);
          
          String status = emergencyState ? "ENABLED" : "DISABLED";
          response = "{\"success\":true,\"message\":\"Emergency mode " + status + "\"}";
          
          // Serial.print("Emergency mode ");
          // Serial.print(status);
          // Serial.println(" via web interface (JSON)");
          
          // Broadcast change as SSE event
          JSONVar statusObj;
          statusObj["emergency"] = emergencyState;
          statusObj["status"] = status;
          events.send(JSON.stringify(statusObj), "emergency_update", millis());
        }
        
        // Free allocated memory
        free(request->_tempObject);
        request->_tempObject = nullptr;
      }
      
      request->send(200, "application/json", response);
    },
    nullptr, // No upload handler needed
    [](AsyncWebServerRequest* request, uint8_t *data, size_t len, size_t index, size_t total) {
      // Store incoming JSON data for processing in the main handler
      if (len && index == 0) {
        // First chunk - allocate memory
        request->_tempObject = malloc(total + 1);
        if (request->_tempObject) {
          memcpy(request->_tempObject, data, len);
          ((char*)request->_tempObject)[len] = 0; // Null terminator
        }
      } else if (len && request->_tempObject) {
        // Subsequent chunks - append data
        memcpy(((char*)request->_tempObject) + index, data, len);
        // Add null terminator if this is the last chunk
        if (index + len >= total) {
          ((char*)request->_tempObject)[total] = 0;
        }
      }
    }
  );

  // Configure SSE events
  events.onConnect([](AsyncEventSourceClient* client) {
    // if (client->lastId()) {
    //   Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
    // }
    client->send("connected", NULL, millis(), 10000);
  });

  // Add event handler and start the server
  server.addHandler(&events);
  AsyncElegantOTA.begin(&server);
  server.begin();
}
