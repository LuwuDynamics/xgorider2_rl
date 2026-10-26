#include "leg_position_controller.h"
#include "policy/policy.h"
#include <algorithm>
#include <cmath>

namespace rider { namespace firmware {
void LegPositionController::reset(uint32_t,float) {
    command_={};legs_={};roll_pid_=control::rollPid();
}
MotionResult LegPositionController::step(const MotionInput& i,MotionRequest& request) {
    MotionResult o;
    if(i.mode!=1||!i.calibrated||i.zero_a>1023||i.zero_b>1023||
       !std::isfinite(i.pitch_deg)||!std::isfinite(i.roll_deg)||!std::isfinite(i.yaw_deg)||
       !std::isfinite(i.pitch_rate_rad_s)||!std::isfinite(i.speed11)||!std::isfinite(i.speed21)||
       !std::isfinite(request.command.max_forward)||!std::isfinite(request.command.max_turn)||
       request.command.max_forward<0||request.command.max_turn<0||
       !std::isfinite(request.roll_phase)||!std::isfinite(request.height_phase)) {
        reset();return o;
    }
    auto command=request.command;
    command.max_forward=std::min(command.max_forward,policy::kMaxForwardMmS);
    command.max_turn=std::min(command.max_turn,policy::kMaxTurnDegS);
    if(command.max_forward==0||command.max_turn==0)command.automatic_lean=false;
    command_.roll_phase=request.roll_phase;command_.height_phase=request.height_phase;
    // Clamp the mapped targets BEFORE filtering, so a phone action cannot wind
    // the filters outside the actor's qualified reference profile.
    const float previous_height=command_.filtered_height;
    control::updateCommandTargets(command_,command);
    command_.roll=std::clamp(command_.roll,-policy::kMaxRollCommandDeg,policy::kMaxRollCommandDeg);
    command_.height=std::clamp(command_.height,0.0f,policy::kMaxHeightUnits);
    command_.filtered_height=static_cast<float>(double(previous_height)*.97+double(command_.height)*.03);
    float roll_increment=0;
    if(request.leveling) {
        roll_pid_.target=0;roll_pid_.measurement=i.roll_deg;
        roll_increment=control::updatePid(roll_pid_);
    }
    control::updateLegTargets(legs_,command_.filtered_height,command_.roll,request.leveling,
                              roll_increment,i.zero_a,i.zero_b);
    o.legs=legs_;o.command=command_;
    o.position=o.physical_position=control::wheelDistance(i.encoder11,i.encoder21);
    o.valid=std::isfinite(o.command.filtered_forward)&&std::isfinite(o.command.filtered_turn)&&
            std::isfinite(o.command.filtered_height)&&std::isfinite(o.legs.roll_target_deg)&&
            o.legs.servo_a<=1023&&o.legs.servo_b<=1023;
    if(!o.valid){reset();return MotionResult{};}
    request.roll_phase=command_.roll_phase;request.height_phase=command_.height_phase;
    return o;
}
}}
