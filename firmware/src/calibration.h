#pragma once

#include <stdint.h>

namespace rider2 {

constexpr uint32_t kLegCalibrationAddress = 0x310000;
constexpr uint32_t kPitchTrimAddress = 0x321000;

struct Calibration {
    uint32_t raw_words[3] = {};
    uint16_t zero_a = 512;
    uint16_t zero_b = 512;
    uint16_t marker = 0;
    uint32_t pitch_trim_low_word = 0;
    // Exact original decode: one flash uint32 copied into the LOW half of an
    // otherwise zero double. This is not a conventional float/double encoding.
    double pitch_trim = 0;
    bool flash_read_ok = false;
    bool marker_valid = false;
    bool zeros_valid = false;
    bool pitch_trim_valid = false;
    int32_t leg_read_error = 0;
    int32_t trim_read_error = 0;
};

// Read-only SPI flash access. Never erases, writes, or auto-calibrates.
Calibration readCalibration();

}  // namespace rider2
