#include "wifi_config.h"
#include "config.h"
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <Arduino.h>

void initWiFi() {
  WiFiManager wifiManager;
  wifiManager.autoConnect(HOSTNAME);

  Serial.println(WiFi.localIP());

  if (!MDNS.begin(MDNS_NAME)) {
    Serial.println("mDNS init failed");
  } else {
    Serial.println("mDNS started");
  }
}
