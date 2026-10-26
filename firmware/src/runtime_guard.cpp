#include "runtime_guard.h"
#include <cmath>
namespace rider { namespace firmware {
const char* stopReasonName(StopReason r) {
    switch(r){
    case StopReason::None:return "none";
    case StopReason::Startup:return "startup";
    case StopReason::Manual:return "manual_stop";
    case StopReason::Calibration:return "calibrating";
    case StopReason::InvalidCalibration:return "invalid_flash_calibration";
    case StopReason::Imu:return "imu_missing_stale_or_nonfinite";
    case StopReason::Actuators:return "actuator_feedback_missing_or_stale";
    case StopReason::Tilt:return "tilt";
    case StopReason::LowBattery:return "low_battery";
    case StopReason::Timing:return "control_gap_over_30ms";
    case StopReason::InvalidControl:return "invalid_control_output";
    case StopReason::Policy:return "policy_fault";
    case StopReason::Fall:return "free_fall";
    }return "unknown";
}
StopReason outputGuard(const GuardInput& i) {
    if(i.manual_stop)return StopReason::Manual;
    if(i.calibrating)return StopReason::Calibration;
    if(i.low_battery)return StopReason::LowBattery;
    if(!i.calibration_valid)return StopReason::InvalidCalibration;
    if(!i.imu_valid||!std::isfinite(i.pitch)||!std::isfinite(i.roll)||uint32_t(i.now_ms-i.imu_ms)>30)return StopReason::Imu;
    if(!i.wheel11_valid||!i.wheel21_valid||!i.leg12_valid||!i.leg22_valid||
       uint32_t(i.now_ms-i.wheel11_ms)>100||uint32_t(i.now_ms-i.wheel21_ms)>100||
       uint32_t(i.now_ms-i.leg12_ms)>250||uint32_t(i.now_ms-i.leg22_ms)>250)return StopReason::Actuators;
    if(i.control_overrun)return StopReason::Timing;
    // Falling: the stop releases the legs and zeroes the wheels before the landing (see fall_guard.h).  Not re-armable
    // while it lasts, so the robot cannot restart in mid-air.
    if(i.falling)return StopReason::Fall;
    // Added immediate guard. The original IMU fault flag uses an AND and a call
    // counter; keep that reconstructed flag too, but do not rely on it alone.
    if(std::fabs(i.pitch)>45||std::fabs(i.roll)>60)return StopReason::Tilt;
    if(i.now_ms<3000||!i.imu_calibrated)return StopReason::Startup;
    return StopReason::None;
}
StopReason stopReasonWhileDriving(StopReason reason, bool driving) {
    return driving && reason == StopReason::Startup ? StopReason::Imu : reason;
}
bool uprightForStart(const GuardInput& i) {return outputGuard(i)==StopReason::None&&std::fabs(i.pitch)<8&&std::fabs(i.roll)<5;}
bool rearmableStop(StopReason r) {
    // Only losses that happen while the actor is driving are recoverable by
    // righting the robot. Manual/emergency stops and low battery stay latched
    // until an explicit arm.
    // Startup covers mode changes caused by handling/power events while the
    // actor is off; Imu/InvalidControl cover transient sensor or control
    // glitches. The phone app has no explicit arm button, so a benign stop must
    // re-arm once the robot is upright, still, and the full guard passes again.
    // Actuators: a servo that stops answering (the leg servo ID 12 did, three
    // times on 2026-10-22, right when the legs got their first command, and it
    // answered again a few seconds later) must not leave the robot dead until a
    // phone/USB command.  The re-arm path only counts ticks while outputGuard()
    // passes, i.e. while every servo is fresh again, so a servo that stays
    // silent never re-arms and nothing moves.
    // Fall: a drop is recoverable once the robot has landed, stands upright and is still (outputGuard() keeps failing while falling).
    // Manual, low-battery and calibration stops still stay latched.
    return r==StopReason::Startup||r==StopReason::Tilt||r==StopReason::Policy||
           r==StopReason::Timing||r==StopReason::Imu||r==StopReason::InvalidControl||
           r==StopReason::Actuators||r==StopReason::Fall;
}
}}
