/*
 * Ground side: transparent ESP-NOW <-> USB serial bridge for cocpitView.
 *
 * Protocol-agnostic by design (doc/architecture.md): it only moves opaque
 * payloads between the two links, adding/stripping the COBS+CRC framing the
 * serial side needs (ESP-NOW preserves packet boundaries, serial does not).
 * Wire-format changes therefore never touch this firmware.
 *
 *   ESP-NOW rx  -> queue -> loop() -> frameEncode -> USB serial (460800)
 *   USB serial  -> FrameDecoder -> loop() -> esp_now_send
 *
 * The UART carries framed binary in both directions, so this firmware never
 * prints anything. The ESP-NOW receive callback runs in the WiFi task; it
 * only queues the payload — Serial is written exclusively from loop().
 *
 * The peer (air board) MAC comes from -DPEER_MAC_STR, injected from
 * [env:air] custom_board_mac by tools/peer_mac.py.
 */

#include <Arduino.h>
#include <esp_now.h>

#include "espnow_link.h"
#include "framing.h"

namespace {

constexpr uint32_t kSerialBaud = 460800;

struct Packet {
  uint8_t len;
  uint8_t data[250];
};

QueueHandle_t rxQueue;
uint8_t peerMac[6];
framing::FrameDecoder decoder;

// WiFi task context: queue only, drop when the loop can't keep up.
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // ESP-NOW hands frames from ANY sender to this callback (encryption
  // only covers the registered peer) - forward only the air board's.
  if (memcmp(info->src_addr, peerMac, 6) != 0) {
    return;
  }
  if (len <= 0 || len > (int)sizeof(Packet::data)) {
    return;
  }
  Packet pkt;
  pkt.len = len;
  memcpy(pkt.data, data, len);
  xQueueSend(rxQueue, &pkt, 0);
}

}  // namespace

void setup() {
  // Default 256 B rx buffer is ~5 ms at 460800 baud; must be set before begin().
  Serial.setRxBufferSize(1024);
  Serial.begin(kSerialBaud);

  rxQueue = xQueueCreate(8, sizeof(Packet));

  if (!espnowlink::parseMac(PEER_MAC_STR, peerMac) || !espnowlink::begin(peerMac)) {
    // Bridge is useless without the radio; stay silent (binary UART) and
    // dead — cocpitView shows NO TELEMETRY, which is the diagnostic.
    while (true) {
      delay(1000);
    }
  }
  esp_now_register_recv_cb(onEspNowReceive);
}

void loop() {
  // ESP-NOW -> USB
  Packet pkt;
  while (xQueueReceive(rxQueue, &pkt, 0) == pdTRUE) {
    uint8_t frame[framing::kMaxFrame];
    size_t n = framing::frameEncode(pkt.data, pkt.len, frame, sizeof(frame));
    if (n > 0) {
      Serial.write(frame, n);
    }
  }

  // USB -> ESP-NOW
  int c;
  while ((c = Serial.read()) >= 0) {
    size_t n = decoder.feed((uint8_t)c);
    if (n > 0) {
      esp_now_send(peerMac, decoder.payload(), n);
    }
  }
}
