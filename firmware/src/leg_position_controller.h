#pragma once
#include "motion_controller.h"

namespace rider { namespace firmware {
// Retains the trained simulation's command filters and leg position kinematics.
// This controller has no wheel feedback loop and always returns zero wheels.
class LegPositionController {
public:
    MotionResult step(const MotionInput&,MotionRequest&);
    void reset(uint32_t now_ms=0,float yaw_deg=0);
private:
    control::CommandState command_{};
    control::LegState legs_{};
    control::Pid roll_pid_=control::rollPid();
};
}}
