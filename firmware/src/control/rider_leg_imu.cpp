#include "rider_leg_imu.h"
#include <cmath>

namespace rider { namespace control {
namespace {
float clamp(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (hi < value) return hi;
    return value;
}
std::int16_t bigSigned(const std::uint8_t *p) {
    const auto value = std::uint16_t((std::uint16_t(p[0]) << 8) | p[1]);
    return value < 0x8000u ? std::int16_t(value)
                          : std::int16_t(std::int32_t(value) - 65536);
}
float acceleration(std::int16_t raw, int sign) {
    const float scaled = float(raw) * float(sign) * 0.000030517578125f;
    const double g = double(scaled) * 9.8;
    return float(g + g);
}
std::uint32_t extensionTicks(float extension) {
    // 0x400d463c..0x400d4690. The acos argument is in [45/96,81/96].
    const double angle = std::acos((double(extension) * 0.5) / 48.0);
    return std::uint32_t(((1.1408376963350786 - angle) / 6.283185307179586) * 1024.0);
}
}

void updateLegTargets(LegState &s, float height, float command_roll_deg,
                      bool leveling, float roll_pid_increment,
                      std::uint16_t zero_a, std::uint16_t zero_b) {
    // 0x400d4514; float stores are retained at original rounding boundaries.
    if (leveling) s.roll_target_deg += roll_pid_increment;
    else s.roll_target_deg = float(double(s.roll_target_deg) * 0.95 +
                                  double(command_roll_deg) * 0.050000000000000044);
    s.roll_target_deg = clamp(s.roll_target_deg, -20.0f, 20.0f);
    const double difference = std::sin(double(s.roll_target_deg) / 57.3) * 43.0;
    const float a = float(double(height) + difference);
    const float b = float(double(height) - difference);
    s.extension_a = clamp(a, 0.0f, 36.0f) + 45.0f;
    s.extension_b = clamp(b, 0.0f, 36.0f) + 45.0f;
    s.servo_a = std::uint16_t(std::uint32_t(zero_a) - extensionTicks(s.extension_a));
    s.servo_b = std::uint16_t(std::uint32_t(zero_b) + extensionTicks(s.extension_b));
}

void offsetLegExtension(LegState &s, float offset_a_m, float offset_b_m,
                        std::uint16_t zero_a, std::uint16_t zero_b) {
    // Convert the actor's metric extension offsets to the firmware's length
    // units and clamp before recomputing servo ticks, matching the MuJoCo
    // position-target model used for training.
    s.extension_a = clamp(s.extension_a + offset_a_m * 1000.0f, 45.0f, 81.0f);
    s.extension_b = clamp(s.extension_b + offset_b_m * 1000.0f, 45.0f, 81.0f);
    s.servo_a = std::uint16_t(std::uint32_t(zero_a) - extensionTicks(s.extension_a));
    s.servo_b = std::uint16_t(std::uint32_t(zero_b) + extensionTicks(s.extension_b));
}

float updateBalanceBias(BalanceBiasState &s, std::uint8_t previous_mode,
                        std::uint8_t mode, bool imu_calibrated,
                        float pitch_deg, float roll_deg, float height,
                        double stored_pitch_trim) {
    // 0x400d4e78. The calibration transient takes precedence over the startup one.
    if (previous_mode == 0 && mode == 1) s.startup_count = 1;
    if (std::uint16_t(s.startup_count - 1u) < 29u) {
        s.transient_bias = pitch_deg < 0.0f ? -8.0f : 8.0f;
        if (std::fabs(pitch_deg) < 10.0f) s.transient_bias = 0.0f;
        ++s.startup_count;
    } else {
        s.transient_bias = 0.0f;
        s.startup_count = 0;
    }
    if (imu_calibrated && s.calibrated_count == 0) s.calibrated_count = 1;
    if (std::uint8_t(s.calibrated_count - 1u) < 24u) {
        s.transient_bias = pitch_deg < 0.0f ? -4.0f : 4.0f;
        ++s.calibrated_count;
    }
    s.roll_bias = -std::fabs(roll_deg) / 14.0f;
    const float h = 1.0f - height / 36.0f;
    double result = double(h) + double(h);
    result += 0.0;
    result += double(s.roll_bias);
    result += double(s.transient_bias);
    result += stored_pitch_trim;
    return s.output = float(result);
}

bool updateImu(ImuState &s, const std::uint8_t *raw, std::size_t size,
               std::uint32_t now_us, std::uint32_t uptime_ms,
               std::uint8_t robot_mode) {
    if (!raw || size != 12) return false;
    s.just_calibrated = false;
    s.tilt_fault = false;
    // 0x400d3190..0x400d3324, raw sensor big-endian words and mounting axes.
    s.acceleration[0] = acceleration(bigSigned(raw), -1);
    s.acceleration[1] = acceleration(bigSigned(raw + 2), -1);
    s.acceleration[2] = acceleration(bigSigned(raw + 4), 1);
    constexpr float scale = 0.000030517578125f;
    s.gyro_deg_s[0] = -float(bigSigned(raw + 8)) * scale * 2000.0f;
    s.gyro_deg_s[1] = float(bigSigned(raw + 6)) * scale * 2000.0f;
    s.gyro_deg_s[2] = float(bigSigned(raw + 10)) * scale * 2000.0f;
    if (s.acceleration[0] == s.last_accel_x && s.acceleration[1] == s.last_accel_y)
        s.identical_samples = s.identical_samples < 500 ? s.identical_samples + 1 : 500;
    else s.identical_samples = 0;
    s.last_accel_x = s.acceleration[0];
    s.last_accel_y = s.acceleration[1];

    // 0x400d3110 called before filtering. Firmware adds 251 readings /250,
    // then arms on the following successful sample; do not silently correct it.
    if (uptime_ms > 750 && !s.calibrated) {
        if (s.calibration_count > 250) s.calibrated = s.just_calibrated = true;
        else {
            if (std::fabs(s.roll_deg) < 4.0f) {
                ++s.calibration_count;
                s.gyro_bias[2] += s.gyro_deg_s[2] / 250.0f;
            } else {
                s.calibration_count = 0;
                s.gyro_bias[2] = 0;
            }
            s.calibration_yaw_deg = s.yaw_deg;
        }
    }
    const float yaw_rate = s.gyro_deg_s[2] - s.gyro_bias[2];
    if (std::fabs(s.gyro_pitch_filtered - s.gyro_deg_s[1]) < 1400.0f)
        s.gyro_pitch_filtered = float(double(s.gyro_deg_s[1] - s.gyro_bias[1]) * 0.7 +
                                      double(s.gyro_pitch_filtered) * 0.3);
    if (std::fabs(s.gyro_roll_filtered - s.gyro_deg_s[0]) < 1800.0f)
        s.gyro_roll_filtered = float(double(s.gyro_deg_s[0] - s.gyro_bias[0]) * 0.93 +
                                     double(s.gyro_roll_filtered) * 0.07);
    s.pitch_rate_rad_s = float(double(s.gyro_pitch_filtered) / 57.3);
    // Firmware uses atanf(y/z), not atan2. Keep the float intermediate.
    const float accel_pitch = float(double(std::atan(s.acceleration[1] / s.acceleration[2])) * 57.2958);
    const float accel_roll = float(double(std::atan(s.acceleration[0] / s.acceleration[2])) * 57.2958);
    s.accel_pitch_filtered = float(double(s.accel_pitch_filtered) * 0.97 + double(accel_pitch) * 0.03);
    s.accel_roll_filtered = float(double(s.accel_roll_filtered) * 0.92 + double(accel_roll) * 0.08);
    const double dt = double(std::uint32_t(now_us - s.last_us)) * 0.000001;
    s.last_us = now_us;
    const double pitch_weight = s.calibrated ? 1.0 / (double(std::fabs(s.gyro_pitch_filtered)) + 500.0) : 0.05;
    const double roll_weight = 1.0 / (double(std::fabs(s.gyro_roll_filtered)) + 200.0);
    s.pitch_deg = float((double(s.pitch_deg) - double(s.gyro_pitch_filtered) * dt) * (1.0 - pitch_weight) +
                        double(s.accel_pitch_filtered) * pitch_weight);
    s.pitch_rad = float(double(s.pitch_deg) / 57.3);
    s.roll_deg = float((double(s.roll_deg) - double(s.gyro_roll_filtered) * dt) * (1.0 - roll_weight) +
                       double(s.accel_roll_filtered) * roll_weight);
    if (std::fabs(yaw_rate) > 2.0f) s.yaw_deg = float(double(s.yaw_deg) + double(yaw_rate) * dt);

    // 0x400d37b1..3839: preserve the firmware's AND between pitch and roll tests.
    // Calling code changes mode to 1 immediately on calibration completion.
    const auto effective_mode = s.just_calibrated ? std::uint8_t(1) : robot_mode;
    if (uptime_ms > 5000 && s.calibrated) {
        if ((s.pitch_deg > 28.0f || s.pitch_deg < -34.0f) && effective_mode == 1 &&
            std::fabs(s.roll_deg) > 60.0f) ++s.tilt_count;
        else s.tilt_count = 0;
    }
    s.tilt_fault = s.tilt_count > 165;
    return true;
}
}} // namespace rider::control
