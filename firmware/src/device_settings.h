#pragma once
#include <cstddef>
#include <cstdint>
namespace rider2 {
struct BleName {
    bool valid = false;
    uint8_t length = 0;
    char suffix[21] = {};
    char full_name[30] = {};
    int32_t read_error = 0;
};
BleName readBleName();
struct TrimSetting {
    bool valid = false, extended = false, extension_invalid = false, defaulted = false;
    double value = 0;
    uint32_t legacy_low_word = 0;
    int32_t read_error = 0;
};
// Full firmware extension at trim-sector +16: magic, version, IEEE64, CRC32.
// Invalid/missing extension falls back to the original low32 + zero-high32.
TrimSetting loadPitchTrim();
// Read/modify/erase/write/verify only the three original settings sectors.
// Caller must stop motors before invoking these synchronous flash operations.
// False means validation, partition overlap, I/O, or verification failed.
bool saveLegCalibration(uint16_t zero_a, uint16_t zero_b);
bool savePitchTrim(double trim);
bool saveBleSuffix(const uint8_t *suffix, size_t length);
}
