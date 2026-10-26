#include "board_imu.h"

#include <cstring>

namespace rider2 {
namespace {
struct RegisterValue { uint8_t reg; uint8_t value; };
// 0x400d30b4: write 0x1f, wait 50 ms, then write the remaining four.
constexpr RegisterValue kInitialization[] = {
    {0x1f, 0x9f}, {0x20, 0x08}, {0x21, 0x68}, {0x23, 0x01}, {0x24, 0x01}
};
}

bool BoardImu::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(kImuAddress);
    Wire.write(reg);
    Wire.write(value);
    diagnostics_.last_i2c_error = Wire.endTransmission(true);
    if (diagnostics_.last_i2c_error != 0) {
        diagnostics_.error = ImuError::I2cWrite;
        return false;
    }
    return true;
}

bool BoardImu::readRegisters(uint8_t reg, uint8_t *out, size_t length) {
    Wire.beginTransmission(kImuAddress);
    Wire.write(reg);
    // Original 0x400d301c sends STOP between setting the register and reading.
    diagnostics_.last_i2c_error = Wire.endTransmission(true);
    if (diagnostics_.last_i2c_error != 0) {
        diagnostics_.error = ImuError::I2cWrite;
        return false;
    }
    const size_t received = Wire.requestFrom(kImuAddress, length, true);
    if (received != length) {
        while (Wire.available()) Wire.read();
        diagnostics_.error = ImuError::ShortRead;
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        if (!Wire.available()) {
            diagnostics_.error = ImuError::ShortRead;
            return false;
        }
        out[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
}

bool BoardImu::begin() {
    diagnostics_ = ImuDiagnostics{};
    std::memset(last_raw_, 0, sizeof last_raw_);
    // 0x400d2f98: I2C controller 0, SDA18/SCL19, both pullups enabled, 400 kHz.
    pinMode(kImuSda, INPUT_PULLUP);
    pinMode(kImuScl, INPUT_PULLUP);
    diagnostics_.bus_started = Wire.begin(kImuSda, kImuScl, kImuBusHz);
    if (!diagnostics_.bus_started) {
        diagnostics_.error = ImuError::BusBegin;
        return false;
    }
    // Bring-up guard: bounded stalls; original timeout was 1000 RTOS ticks.
    Wire.setTimeOut(20);
    for (size_t i = 0; i < sizeof kInitialization / sizeof kInitialization[0]; ++i) {
        if (!writeRegister(kInitialization[i].reg, kInitialization[i].value)) return false;
        if (i == 0) delay(50);
    }
    // Additional bring-up check, not present in the original initialization.
    // Unexpected readback stops sampling instead of trying alternate mappings.
    for (size_t i = 0; i < sizeof kInitialization / sizeof kInitialization[0]; ++i) {
        if (!readRegisters(kInitialization[i].reg, &diagnostics_.register_readback[i], 1)) return false;
        if (diagnostics_.register_readback[i] != kInitialization[i].value) {
            diagnostics_.error = ImuError::ReadbackMismatch;
            return false;
        }
    }
    diagnostics_.configured = true;
    diagnostics_.error = ImuError::None;
    return true;
}

bool BoardImu::read(uint8_t raw[12]) {
    diagnostics_.last_read_ok = false;
    if (!diagnostics_.configured || !raw) {
        diagnostics_.error = ImuError::NotConfigured;
        ++diagnostics_.read_failures;
        return false;
    }
    uint8_t sample[12];
    if (!readRegisters(kImuDataRegister, sample, sizeof sample)) {
        ++diagnostics_.read_failures;
        return false;
    }
    bool all_zero = true;
    bool all_ff = true;
    for (const uint8_t value : sample) {
        all_zero = all_zero && value == 0;
        all_ff = all_ff && value == 0xff;
    }
    if (all_zero || all_ff) {
        diagnostics_.error = ImuError::BlankData;
        ++diagnostics_.blank_samples;
        ++diagnostics_.read_failures;
        return false;
    }
    if (diagnostics_.valid_samples && std::memcmp(sample, last_raw_, sizeof sample) == 0) {
        if (diagnostics_.identical_samples != UINT32_MAX) ++diagnostics_.identical_samples;
    } else {
        diagnostics_.identical_samples = 0;
    }
    std::memcpy(last_raw_, sample, sizeof sample);
    std::memcpy(raw, sample, sizeof sample);
    diagnostics_.last_good_us = micros();
    ++diagnostics_.valid_samples;
    diagnostics_.last_read_ok = true;
    diagnostics_.error = ImuError::None;
    return true;
}

const char *imuErrorName(ImuError error) {
    switch (error) {
    case ImuError::None: return "ok";
    case ImuError::BusBegin: return "bus_begin";
    case ImuError::I2cWrite: return "i2c_write";
    case ImuError::ShortRead: return "short_read";
    case ImuError::ReadbackMismatch: return "readback_mismatch";
    case ImuError::BlankData: return "blank_data";
    case ImuError::NotConfigured: return "not_configured";
    }
    return "unknown";
}

}  // namespace rider2
