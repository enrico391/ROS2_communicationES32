#pragma once
#include <Wire.h>

// Adafruit_MPU6050::getEvent() currently returns true even when its underlying
// I2C read fails. Observe those transactions so a failed event never becomes a
// fresh sample. Concurrent users must hold the external I2C mutex; a standalone
// diagnostic loop can own the bus directly.
class CheckedWire : public TwoWire {
public:
  CheckedWire() : TwoWire(0) {}
  void clearError() { failed_ = false; }
  bool failed() const { return failed_; }
  uint8_t endTransmission(bool stop) override {
    const uint8_t result = TwoWire::endTransmission(stop);
    failed_ = failed_ || result != 0;
    return result;
  }
  uint8_t endTransmission() override { return endTransmission(true); }
  size_t requestFrom(uint8_t address, size_t count, bool stop) override {
    const size_t received = TwoWire::requestFrom(address, count, stop);
    failed_ = failed_ || received != count;
    return received;
  }
  size_t requestFrom(uint8_t address, size_t count) override {
    return requestFrom(address, count, true);
  }
private:
  bool failed_ = false;
};
