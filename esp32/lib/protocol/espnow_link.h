// Shared ESP-NOW bring-up for both firmwares: STA mode, long-range-only
// protocol, fixed channel, encryption (keys from the -DLINK_PMK_HEX /
// -DLINK_LMK_HEX build flags, see tools/link_keys.py) and the single peer.
// Pure link layer — message structs stay out so the ground bridge remains
// protocol-agnostic. Callbacks are registered by the caller via esp_now_*.

#pragma once

#include <stdint.h>

namespace espnowlink {

// Parses "68:09:47:f6:b9:b0" (the -DPEER_MAC_STR build flag) into 6 bytes.
bool parseMac(const char *str, uint8_t out[6]);

// Parses 32 hex digits (the -DLINK_*_HEX build flags) into 16 bytes.
bool parseKey(const char *str, uint8_t out[16]);

// Full radio bring-up; returns false on any esp_err_t failure — the reason
// is then available from lastError() (the caller decides whether printing
// is safe: the air side prints '#' diagnostics, the ground stays silent).
bool begin(const uint8_t peerMac[6]);

const char *lastError();

}  // namespace espnowlink
