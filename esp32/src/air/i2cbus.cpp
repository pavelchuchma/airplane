#include "i2cbus.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>

namespace i2cbus {
namespace {

constexpr gpio_num_t kSda = GPIO_NUM_21;
constexpr gpio_num_t kScl = GPIO_NUM_22;
// 100 kHz, not 400: 400 kHz is the MS4525DO's spec maximum, and that device
// hangs on a wire harness off the GY-91 board. ~30 B per 20 ms tick.
constexpr uint32_t kClockHz = 100000;
// The longest read (MPU, 14 B) takes ~2 ms; a held SCL costs exactly this
// (i2ctest), so it must stay well under the 20 ms sample period.
constexpr int kTimeoutMs = 5;
constexpr size_t kMaxDevices = 4;
constexpr size_t kMaxWrite = 8;  // register + payload of writeRegs

i2c_master_bus_handle_t bus = nullptr;
struct Device {
  uint8_t addr;
  i2c_master_dev_handle_t handle;
};
Device devices[kMaxDevices];
size_t deviceCount = 0;

Status status = Status::Ok;
bool cleared = false;

bool linesIdle() {
  return gpio_get_level(kSda) && gpio_get_level(kScl);
}

i2c_master_dev_handle_t device(uint8_t addr) {
  for (size_t i = 0; i < deviceCount; i++) {
    if (devices[i].addr == addr) {
      return devices[i].handle;
    }
  }
  if (bus == nullptr || deviceCount == kMaxDevices) {
    return nullptr;
  }
  i2c_device_config_t conf = {};
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.device_address = addr;
  conf.scl_speed_hz = kClockHz;
  i2c_master_dev_handle_t handle;
  if (i2c_master_bus_add_device(bus, &conf, &handle) != ESP_OK) {
    return nullptr;
  }
  devices[deviceCount++] = {addr, handle};
  return handle;
}

// Gets the bus ready for a START: a held line is cleared first, and if that
// does not free it the transaction is not attempted at all.
i2c_master_dev_handle_t start(uint8_t addr) {
  cleared = false;
  if (!linesIdle() && !clear()) {
    status = Status::BusBusy;
    return nullptr;
  }
  i2c_master_dev_handle_t handle = device(addr);
  if (handle == nullptr) {
    status = Status::Failed;
  }
  return handle;
}

Status finish(esp_err_t err) {
  status = err == ESP_OK            ? Status::Ok
           : err == ESP_ERR_TIMEOUT ? Status::Timeout
                                    : Status::Failed;
  if (!linesIdle()) {
    clear();
    cleared = true;
  }
  return status;
}

}  // namespace

const char *statusName(Status s) {
  switch (s) {
    case Status::Ok: return "ok";
    case Status::BusBusy: return "bus-busy";
    case Status::Failed: return "failed";
    case Status::Timeout: return "timeout";
  }
  return "?";
}

void begin() {
  i2c_master_bus_config_t conf = {};
  conf.i2c_port = I2C_NUM_0;
  conf.sda_io_num = kSda;
  conf.scl_io_num = kScl;
  conf.clk_source = I2C_CLK_SRC_DEFAULT;
  conf.glitch_ignore_cnt = 7;
  conf.flags.enable_internal_pullup = 1;
  if (i2c_new_master_bus(&conf, &bus) != ESP_OK) {
    bus = nullptr;
  }
}

bool clear(bool *wasStuck) {
  bool stuck = !gpio_get_level(kSda);
  if (wasStuck) {
    *wasStuck = stuck;
  }
  if (stuck && bus != nullptr) {
    i2c_master_bus_reset(bus);
  }
  return linesIdle();
}

Status writeRegs(uint8_t addr, uint8_t reg, const uint8_t *data, size_t n) {
  i2c_master_dev_handle_t handle = start(addr);
  if (handle == nullptr) {
    return status;
  }
  uint8_t buf[kMaxWrite];
  n = min(n, kMaxWrite - 1);
  buf[0] = reg;
  memcpy(buf + 1, data, n);
  return finish(i2c_master_transmit(handle, buf, n + 1, kTimeoutMs));
}

Status readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  i2c_master_dev_handle_t handle = start(addr);
  if (handle == nullptr) {
    return status;
  }
  return finish(i2c_master_transmit_receive(handle, &reg, 1, buf, n, kTimeoutMs));
}

Status read(uint8_t addr, uint8_t *buf, size_t n) {
  i2c_master_dev_handle_t handle = start(addr);
  if (handle == nullptr) {
    return status;
  }
  return finish(i2c_master_receive(handle, buf, n, kTimeoutMs));
}

Status lastStatus() {
  return status;
}

bool lastCleared() {
  return cleared;
}

}  // namespace i2cbus
