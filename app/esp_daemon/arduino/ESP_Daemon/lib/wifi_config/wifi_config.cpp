#include "wifi_config.h"
#include "config.h"
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <Arduino.h>

int wifi_channel = 0;

void initWiFi() {
  WiFiManager wifiManager;
  wifiManager.setEnableConfigPortal(false);

  const int maxRetries = 10;
  int retryCount = 0;
  bool connected = false;

  while (retryCount < maxRetries && !connected) {
    Serial.printf("Attempting to connect to WiFi (%d/%d)...\n", retryCount + 1, maxRetries);
    if (wifiManager.autoConnect(HOSTNAME)) {
      connected = true;
      Serial.println("WiFi connected!");
    } else {
      Serial.println("WiFi connection failed, retrying...");
      retryCount++;
      delay(3000);
    }
  }

  if (!connected) {
    Serial.println("Entering WiFi configuration portal...");
    wifiManager.setEnableConfigPortal(true);
    wifiManager.startConfigPortal(HOSTNAME);
  }

  Serial.println(WiFi.localIP());

  if (!MDNS.begin(MDNS_NAME)) {
    Serial.println("mDNS init failed");
  } else {
    Serial.println("mDNS started");
  }
  wifi_channel = WiFi.channel();
}
