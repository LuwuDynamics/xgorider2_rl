#include "calibration.h"

#include <cmath>
#include <cstring>
#include <esp_spi_flash.h>

namespace rider2 {

Calibration readCalibration() {
    static_assert(sizeof(double) == 8, "Recovered trim requires a 64-bit double");
    Calibration result;
    // 0x400d2730 reads count*4 bytes, truncating each LE word to uint16.
    result.leg_read_error = spi_flash_read(kLegCalibrationAddress,
                                          result.raw_words, sizeof result.raw_words);
    result.trim_read_error = spi_flash_read(kPitchTrimAddress,
                                           &result.pitch_trim_low_word, sizeof(uint32_t));
    result.flash_read_ok = result.leg_read_error == ESP_OK && result.trim_read_error == ESP_OK;
    if (result.leg_read_error == ESP_OK) {
        result.marker = static_cast<uint16_t>(result.raw_words[2]);
        result.marker_valid = result.marker == 1234;
        const uint16_t stored_a = static_cast<uint16_t>(result.raw_words[0]);
        const uint16_t stored_b = static_cast<uint16_t>(result.raw_words[1]);
        // Engineering range guard; original only checked marker == 1234.
        result.zeros_valid = result.marker_valid && stored_a <= 1023 && stored_b <= 1023;
        if (result.zeros_valid) {
            result.zero_a = stored_a;
            result.zero_b = stored_b;
        }
    }
    if (result.trim_read_error == ESP_OK) {
        // 0x400d2878 zeros both words, then requests FOUR bytes at 0x321000.
        // Preserve that observable behavior; do not silently read eight bytes.
        const uint64_t bits = result.pitch_trim_low_word;
        std::memcpy(&result.pitch_trim, &bits, sizeof bits);
        result.pitch_trim_valid = result.pitch_trim_low_word != UINT32_MAX &&
                                 std::isfinite(result.pitch_trim) &&
                                 std::fabs(result.pitch_trim) <= 7.5;
        if (!result.pitch_trim_valid) result.pitch_trim = 0;
    }
    return result;
}

}  // namespace rider2
