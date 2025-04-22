#include "espnow_comm.h"
#include "config.h"

#include "wifi_config.h"

#include <WiFi.h>
#include <esp_now.h>
#include <vector>
#include <array>

struct_message myData;
std::vector<std::array<uint8_t, 6>> broadcastAddresses = {SIMA_01, SIMA_02, SIMA_03, SIMA_04};
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

  for (auto& address : broadcastAddresses) {
    memcpy(peerInfo.peer_addr, address.data(), 6);
    peerInfo.channel = wifi_channel;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("ESP-NOW peer add failed");
    }
  }
}

void sendESPNow(int data, int addressIndex) {
  myData.sima_start = data;
  const uint8_t* address = broadcastAddresses[addressIndex].data();
  esp_err_t result = esp_now_send(address, (uint8_t*)&myData, sizeof(myData));
  // Serial.println(result == ESP_OK ? " Success" : " Error");
}

// Overload with single parameter for backward compatibility
void sendESPNow(int data) {
  // By default, send to the first device in the broadcast list
  sendESPNow(data, 0);
}
