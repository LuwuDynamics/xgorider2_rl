#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace rider2 {

// All bus constants and register writes are recovered from Rider v1.1.6.
// No chip identity is inferred from the I2C address alone.
constexpr int kImuSda = 18;
constexpr int kImuScl = 19;
constexpr uint32_t kImuBusHz = 400000;
constexpr uint8_t kImuAddress = 0x69;
constexpr uint8_t kImuDataRegister = 0x0b;

enum class ImuError : uint8_t {
    None, BusBegin, I2cWrite, ShortRead, ReadbackMismatch, BlankData, NotConfigured
};

struct ImuDiagnostics {
    bool bus_started = false;
    bool configured = false;
    bool last_read_ok = false;
    ImuError error = ImuError::NotConfigured;
    uint8_t last_i2c_error = 0;
    // Corresponds to registers 0x1f, 0x20, 0x21, 0x23, 0x24.
    uint8_t register_readback[5] = {};
    uint32_t valid_samples = 0;
    uint32_t read_failures = 0;
    uint32_t blank_samples = 0;
    uint32_t identical_samples = 0;
    uint32_t last_good_us = 0;
};

class BoardImu {
public:
    bool begin();
    // Output is untouched on failure. Caller must skip fusion/shadow on false.
    bool read(uint8_t raw[12]);
    const ImuDiagnostics &diagnostics() const { return diagnostics_; }

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *out, size_t length);
    ImuDiagnostics diagnostics_;
    uint8_t last_raw_[12] = {};
};

const char *imuErrorName(ImuError error);

}  // namespace rider2
