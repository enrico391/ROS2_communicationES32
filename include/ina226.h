#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#ifndef INA226_ADDRESS
#define INA226_ADDRESS 0x40
#endif
#ifndef INA226_SHUNT_OHMS
#define INA226_SHUNT_OHMS 0.002f
#endif
static_assert(INA226_ADDRESS >= 0x40 && INA226_ADDRESS <= 0x4F,
              "INA226 address must be between 0x40 and 0x4F");
static_assert(INA226_SHUNT_OHMS > 0, "INA226 shunt resistance must be positive");
constexpr uint8_t INA226_I2C_ADDRESS = INA226_ADDRESS;
constexpr float INA226_SHUNT_RESISTANCE_OHMS = INA226_SHUNT_OHMS;

// Register definitions and scaling: https://www.ti.com/lit/ds/symlink/ina226.pdf
enum class Ina226Status { Ok, I2cError, WrongDevice, ConfigurationError, NotReady, Overflow };

struct Ina226Sample {
  float bus_V, shunt_mV, current_mA, power_W;
};

// The bus is supplied so the real register transactions can also run in native
// tests. Main firmware and the standalone debug sketch share this driver.
template<class Bus>
class Ina226 {
public:
  Ina226(Bus &bus, uint8_t address, float shuntOhms)
      : bus_(bus), address_(address), shuntOhms_(shuntOhms) {}

  uint16_t manufacturerId = 0, deviceId = 0;

  Ina226Status begin() {
    manufacturerId = deviceId = 0;
    if (!(shuntOhms_ > 0) || !std::isfinite(shuntOhms_))
      return Ina226Status::ConfigurationError;
    if (!readRegister(0xFE, manufacturerId) || !readRegister(0xFF, deviceId))
      return Ina226Status::I2cError;
    if (manufacturerId != 0x5449 || (deviceId & 0xFFF0) != 0x2260)
      return Ina226Status::WrongDevice;
    if (!writeRegister(0x00, configuration_) || !writeRegister(0x05, calibration_))
      return Ina226Status::I2cError;
    return checkConfiguration();
  }

  Ina226Status read(Ina226Sample &sample) {
    const Ina226Status config = checkConfiguration();
    if (config != Ina226Status::Ok) return config;
    uint16_t mask, shunt, bus, current, power;
    if (!readRegister(0x06, mask)) return Ina226Status::I2cError;
    if (mask & 0x0004) return Ina226Status::Overflow;
    if (!(mask & 0x0008)) return Ina226Status::NotReady;
    if (!readRegister(0x01, shunt) || !readRegister(0x02, bus) ||
        !readRegister(0x04, current) || !readRegister(0x03, power))
      return Ina226Status::I2cError;
    // CAL=2048 gives Current_LSB = 0.00512/(2048*R) = 2.5 uV/R.
    // This uses the full signed shunt range without current-register overflow.
    const float currentLsb_A = 0.0000025f / shuntOhms_;
    sample = {bus * 0.00125f, signedValue(shunt) * 0.0025f,
              signedValue(current) * currentLsb_A * 1000.0f,
              power * 25.0f * currentLsb_A};
    return Ina226Status::Ok;
  }

private:
  // 16 averages, 1.1 ms bus + shunt conversions, continuous mode (~35.2 ms).
  static constexpr uint16_t configuration_ = 0x4527;
  static constexpr uint16_t calibration_ = 2048;
  Bus &bus_;
  uint8_t address_;
  float shuntOhms_;

  static int32_t signedValue(uint16_t value) {
    return value & 0x8000 ? int32_t(value) - 65536 : value;
  }

  bool readRegister(uint8_t reg, uint16_t &value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom(address_, size_t(2)) != 2) return false;
    const int high = bus_.read();
    const int low = bus_.read();
    if (high < 0 || low < 0) return false;
    value = (uint16_t(high) << 8) | uint16_t(low);
    return true;
  }

  bool writeRegister(uint8_t reg, uint16_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(uint8_t(value >> 8));
    bus_.write(uint8_t(value));
    return bus_.endTransmission() == 0;
  }

  Ina226Status checkConfiguration() {
    uint16_t config, calibration;
    if (!readRegister(0x00, config) || !readRegister(0x05, calibration))
      return Ina226Status::I2cError;
    return config == configuration_ && calibration == calibration_
        ? Ina226Status::Ok : Ina226Status::ConfigurationError;
  }
};
