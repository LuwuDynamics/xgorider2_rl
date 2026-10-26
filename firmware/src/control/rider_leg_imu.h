#pragma once
#include <cstddef>
#include <cstdint>

namespace rider { namespace control {

// Numeric kernel reconstructed from Rider v1.1.6, 0x400d4514.
// A and B mean the original object order, not an unverified physical side.
struct LegState {
    float roll_target_deg = 0;              // command structure 3ffc30b4 +12
    float extension_a = 45, extension_b = 45; // +16,+20; firmware length units
    std::uint16_t servo_a = 0, servo_b = 0; // 3ffbe170+2, 3ffbe15c+2
};
// roll_pid_increment is updatePid(roll PID)'s output when leveling is enabled.
// The caller sets its target=0 and feedback=measured roll (IMU +96).
void updateLegTargets(LegState &state, float height, float command_roll_deg,
                      bool leveling, float roll_pid_increment,
                      std::uint16_t zero_a, std::uint16_t zero_b);
// Actor leg channels: add a metric extension offset to each leg target and
// recompute the servo tick values with the original acos linkage mapping.
// Offsets are clamped so the extension stays inside the 45..81 mm range.
void offsetLegExtension(LegState &state, float offset_a_m, float offset_b_m,
                        std::uint16_t zero_a, std::uint16_t zero_b);

// 0x400d4e78. Counters are call counts, not milliseconds.
struct BalanceBiasState {
    std::uint16_t startup_count = 0; // 3ffc365e
    std::uint8_t calibrated_count = 0; // 3ffc365c
    float transient_bias = 0;       // 3ffc3650
    float roll_bias = 0;            // 3ffc3654
    float output = 0;               // 3ffc3658
};
float updateBalanceBias(BalanceBiasState &state, std::uint8_t previous_mode,
                        std::uint8_t mode, bool imu_calibrated,
                        float pitch_deg, float roll_deg, float height,
                        double stored_pitch_trim);

struct ImuState {
    float acceleration[3]{}; // body axes: [-raw AX,-raw AY,+raw AZ], m/s^2
    float gyro_deg_s[3]{};   // body axes: [-raw GY,+raw GX,+raw GZ]
    float gyro_bias[3]{};    // original IMU +60,+64,+68; only yaw calibrated here
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0; // IMU +96,+100,+104
    float pitch_rad = 0, pitch_rate_rad_s = 0;    // 3ffc3080 +8,+12
    float calibration_yaw_deg = 0;               // 3ffc3064
    float gyro_roll_filtered = 0, gyro_pitch_filtered = 0;
    float accel_roll_filtered = 0, accel_pitch_filtered = 0;
    float last_accel_x = 0, last_accel_y = 0;
    std::uint16_t identical_samples = 0, calibration_count = 0, tilt_count = 0;
    std::uint32_t last_us = 0;
    bool calibrated = false, just_calibrated = false, tilt_fault = false;
};
// raw: 12-byte big-endian sensor register window 0x0b..0x16.
// Returns false only for an invalid caller buffer; performs no I2C or motor I/O.
// just_calibrated requests mode=1; tilt_fault requests the original stop action.
bool updateImu(ImuState &state, const std::uint8_t *raw, std::size_t size,
               std::uint32_t now_us, std::uint32_t uptime_ms,
               std::uint8_t robot_mode);

}} // namespace rider::control
