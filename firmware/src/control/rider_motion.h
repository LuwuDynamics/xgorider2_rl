#pragma once
#include "pid.h"
#include <cstdint>

// Portable numeric reconstruction of xgorider_app_v1.1.6.bin.
// Each routine cites its original function; this is not a board firmware.
namespace rider { namespace control {

struct MotionCommand {
    std::uint8_t forward=128, turn=128, height=128, roll=128;
    std::uint8_t roll_period=128, height_period=128;
    float max_forward=210, max_turn=300;
    bool automatic_lean=false;
};
struct CommandState {
    float forward=0, turn=0, height=0, roll=0; // mapped requests, 3ffc30d8
    float filtered_forward=0, filtered_turn=0, filtered_height=0;
    float roll_phase=0, height_phase=0; // zero disables periodic movement
};
// 0x400d4248 and first part of 0x400d56cc. Phase increments are 10 per
// invocation; they are not a measured timebase. No dt is introduced.
void updateCommandTargets(CommandState &, const MotionCommand &);

struct HoldState {
    std::uint8_t mode=0; // 0 moving, 1 waiting, 2 holding
    std::uint32_t began_ms=0, last_disabled_ms=0;
    float position=0, correction=0;
};
// 0x400d5288. Feedback position uses wheelDistance(), in metres.
void updatePositionHold(HoldState &, std::uint32_t now_ms,
                        std::uint8_t robot_mode, float forward_request,
                        std::uint8_t turn_byte, float position_m,
                        float pitch_deg, bool protection=true);

struct BalanceState { Pid speed=speedPid(), pitch=pitchPid(); };
struct Wheels { std::int16_t id11=0, id21=0; };
float wheelDistance(std::int32_t id11_encoder, std::int32_t second_encoder);
// 0x400d582c numeric portion. Set forward_target from the PREVIOUS cycle's
// filtered_forward + hold.correction. speed feedback has already been decoded
// and filtered by the serial receiver. Positive pitch_rate is IMU +3ffc308c.
float updateBalance(BalanceState &, float forward_target,
                    float speed11, float speed21, float pitch_deg,
                    float pitch_bias, float pitch_rate_rad_s,
                    float filtered_height, bool holding, Wheels &);
// 0x400d4fd4; called before the command filters are advanced.
void addWheelSpeedCorrection(Wheels &, float filtered_forward,
                             float filtered_turn, float speed11, float speed21);

struct HeadingState {
    Pid pid=headingPid();
    float target=0;
    std::uint32_t last_motion_ms=0;
};
// 0x400d53a4; called after command filtering and leg roll calculation.
void addHeadingCorrection(HeadingState &, Wheels &, std::uint32_t now_ms,
                          std::uint8_t robot_mode, bool hold_heading,
                          float forward_request, float turn_request,
                          float filtered_turn, float yaw_deg,
                          bool leveling, float filtered_roll_deg);
// 0x400d5238. Final +/-15 friction compensation outside strict +/-10.
void addWheelFriction(Wheels &);

}} // namespace rider::control
