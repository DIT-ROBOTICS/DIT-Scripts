#include "espnow_comm.h"
#include "config.h"
#include <WiFi.h>

struct_message myData;
uint8_t broadcastAddress[] = BROADCAST_ADDR;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void initESPNow(){
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
  }
}

void sendESPNow(int mode) {
  myData.sima_start = mode;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
  Serial.println(result == ESP_OK ? "ESP-NOW Success" : "ESP-NOW Error");
}
