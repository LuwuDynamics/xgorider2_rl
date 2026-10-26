#pragma once
#include <cstdint>
namespace rider { namespace firmware {
enum class StopReason : uint8_t { None,Startup,Manual,Calibration,InvalidCalibration,Imu,Actuators,Tilt,LowBattery,Timing,InvalidControl,Policy,Fall };
const char* stopReasonName(StopReason);
struct GuardInput {
    uint32_t now_ms=0,imu_ms=0,wheel11_ms=0,wheel21_ms=0,leg12_ms=0,leg22_ms=0;
    bool imu_valid=false,imu_calibrated=false,calibration_valid=false;
    bool wheel11_valid=false,wheel21_valid=false,leg12_valid=false,leg22_valid=false;
    bool manual_stop=false,calibrating=false,low_battery=false;
    bool control_overrun=false;
    bool falling=false;      // free fall detected (src/fall_guard.h): servos must let go before the impact
    float pitch=0,roll=0;
};
// Independent of modes/actions: every actual motor output must pass this guard.
StopReason outputGuard(const GuardInput&);
bool uprightForStart(const GuardInput&);
// While the robot is driving, "startup" can only mean the IMU fusion restarted (not calibrated any more): report it as the IMU stop it is,
// so it is not silent (the LED shows nothing for a plain startup).
StopReason stopReasonWhileDriving(StopReason reason, bool driving);
// Automatic stops that may re-select the learned controller once the robot is
// set back upright. Manual, low-battery, calibration, sensor and actuator stops
// always require an explicit arm.
bool rearmableStop(StopReason);
}}
