#pragma once
#include "control/rider_motion.h"
#include "control/rider_leg_imu.h"

namespace rider { namespace firmware {
struct MotionRequest {
    control::MotionCommand command{};
    float roll_phase=0,height_phase=0;
    bool leveling=false,heading_hold=true,hold_protection=true;
    float heading_term_limit=300;
    // The original 0x400d58fc uses leg12.position, not wheel21.encoder.
    // Keep the known instruction behavior explicit instead of silently fixing it.
    bool original_position_feedback=true;
};
struct MotionInput {
    uint32_t now_ms=0;
    uint8_t mode=0;
    bool calibrated=false;
    float pitch_deg=0,roll_deg=0,yaw_deg=0,pitch_rate_rad_s=0;
    float speed11=0,speed21=0;
    int32_t encoder11=0,encoder21=0;
    int16_t leg12_position=0;
    uint16_t zero_a=512,zero_b=512;
    double pitch_trim=0;
};
struct MotionResult {
    bool valid=false;
    control::Wheels wheels{};
    control::LegState legs{};
    control::CommandState command{};
    control::HoldState hold{};
    float balance=0,bias=0,position=0,physical_position=0;
};
class MotionController {
public:
    MotionResult step(const MotionInput&,MotionRequest&);
    void reset(uint32_t now_ms=0,float yaw_deg=0);
    // Action reset (0x400d4740) preserves filters/PID histories and only resets
    // the position-hold mode and its last-disabled timestamp.
    void resetHold(uint32_t now_ms);
    // Stopped-mode 0x400d47f0 clears exactly these two PID history fields.
    // Actual wheel zero output is issued by the runtime/actuator stop path.
    void resetStoppedPid();
    float holdCorrection() const {return hold_.correction;}
    uint8_t holdMode() const {return hold_.mode;}
private:
    control::BalanceState balance_{};
    control::CommandState command_{};
    control::BalanceBiasState bias_{};
    control::LegState legs_{};
    control::HeadingState heading_{};
    control::HoldState hold_{};
    control::Pid roll_pid_=control::rollPid();
    float next_forward_=0;
    uint8_t previous_mode_=0;
};
}}
