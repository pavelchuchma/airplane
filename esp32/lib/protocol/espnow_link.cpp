#include "espnow_link.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <string.h>

#include "link_keys.h"
#include "messages.h"  // WIFI_CHANNEL

namespace espnowlink {

namespace {
const char *lastError_ = "";
}

bool parseMac(const char *str, uint8_t out[6]) {
  return sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                &out[0], &out[1], &out[2], &out[3], &out[4], &out[5]) == 6;
}

bool begin(const uint8_t peerMac[6]) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // The default modem power save drops ESP-NOW frames.
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    lastError_ = "esp_wifi_set_ps failed";
    return false;
  }
  // Long-range-only: mandatory for flying range, and it also means the
  // frames are invisible to normal 802.11 gear (debugging note).
  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK) {
    lastError_ = "esp_wifi_set_protocol(LR) failed";
    return false;
  }
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    lastError_ = "esp_wifi_set_channel failed";
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    lastError_ = "esp_now_init failed";
    return false;
  }
  // PMK must be set after esp_now_init and before adding encrypted peers.
  if (esp_now_set_pmk(LINK_PMK) != ESP_OK) {
    lastError_ = "esp_now_set_pmk failed";
    return false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;  // CCMP in hardware; key mismatch fails silently
  memcpy(peer.lmk, LINK_LMK, sizeof(peer.lmk));
  if (esp_now_add_peer(&peer) != ESP_OK) {
    lastError_ = "esp_now_add_peer failed";
    return false;
  }
  return true;
}

const char *lastError() { return lastError_; }

}  // namespace espnowlink
