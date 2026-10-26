#include "motion_controller.h"
#include <algorithm>
#include <cmath>

namespace rider { namespace firmware {
void MotionController::reset(uint32_t now,float yaw) {
    balance_={};command_={};bias_={};legs_={};heading_={};hold_={};roll_pid_=control::rollPid();
    next_forward_=0;previous_mode_=0;heading_.target=yaw;heading_.last_motion_ms=now;
    hold_.last_disabled_ms=now;
}
void MotionController::resetHold(uint32_t now) {
    hold_.mode=0;hold_.last_disabled_ms=now;
}
void MotionController::resetStoppedPid() {
    balance_.speed.previous_error=balance_.speed.integral=0;
    balance_.pitch.previous_error=balance_.pitch.integral=0;
}
MotionResult MotionController::step(const MotionInput& i,MotionRequest& request) {
    MotionResult o;
    if(i.mode!=1 || !i.calibrated || i.zero_a>1023 || i.zero_b>1023 ||
       !std::isfinite(i.pitch_deg)||!std::isfinite(i.roll_deg)||!std::isfinite(i.yaw_deg)||
       !std::isfinite(i.pitch_rate_rad_s)||!std::isfinite(i.speed11)||!std::isfinite(i.speed21)||
       !std::isfinite(i.pitch_trim)||!std::isfinite(request.command.max_forward)||
       !std::isfinite(request.command.max_turn)||request.command.max_forward<0||request.command.max_turn<0||
       !std::isfinite(request.heading_term_limit)||request.heading_term_limit<0||
       !std::isfinite(request.roll_phase)||!std::isfinite(request.height_phase)) {
        reset(i.now_ms,std::isfinite(i.yaw_deg)?i.yaw_deg:0);return o;
    }
    auto command=request.command;
    // A zero speed-limit byte is legal. Avoid dividing by zero in the dormant
    // automatic lean branch while retaining the zero movement limit.
    if(command.max_forward==0||command.max_turn==0)command.automatic_lean=false;
    command_.roll_phase=request.roll_phase;command_.height_phase=request.height_phase;
    o.physical_position=control::wheelDistance(i.encoder11,i.encoder21);
    o.position=control::wheelDistance(i.encoder11,request.original_position_feedback?int32_t(i.leg12_position):i.encoder21);
    o.bias=control::updateBalanceBias(bias_,previous_mode_,i.mode,i.calibrated,
                                    i.pitch_deg,i.roll_deg,command_.filtered_height,i.pitch_trim);
    previous_mode_=i.mode;
    o.balance=control::updateBalance(balance_,next_forward_,i.speed11,i.speed21,
              i.pitch_deg,o.bias,i.pitch_rate_rad_s,command_.filtered_height,hold_.mode==2,o.wheels);
    control::addWheelSpeedCorrection(o.wheels,command_.filtered_forward,command_.filtered_turn,i.speed11,i.speed21);
    control::updateCommandTargets(command_,command);
    control::updatePositionHold(hold_,i.now_ms,i.mode,command_.forward,command.turn,o.position,
                                i.pitch_deg,request.hold_protection);
    next_forward_=command_.filtered_forward+hold_.correction;
    float roll_increment=0;
    if(request.leveling){roll_pid_.target=0;roll_pid_.measurement=i.roll_deg;roll_increment=control::updatePid(roll_pid_);}
    control::updateLegTargets(legs_,command_.filtered_height,command_.roll,request.leveling,
                              roll_increment,i.zero_a,i.zero_b);
    heading_.pid.p_limit=heading_.pid.i_limit=heading_.pid.d_limit=request.heading_term_limit;
    control::addHeadingCorrection(heading_,o.wheels,i.now_ms,i.mode,request.heading_hold,
        command_.forward,command_.turn,command_.filtered_turn,i.yaw_deg,request.leveling,legs_.roll_target_deg);
    control::addWheelFriction(o.wheels);
    o.wheels.id11=std::max<int16_t>(-1023,std::min<int16_t>(1023,o.wheels.id11));
    o.wheels.id21=std::max<int16_t>(-1023,std::min<int16_t>(1023,o.wheels.id21));
    o.legs=legs_;o.command=command_;o.hold=hold_;
    o.valid=std::isfinite(o.balance)&&std::isfinite(o.bias)&&std::isfinite(next_forward_)&&
            std::isfinite(o.command.filtered_height)&&o.legs.servo_a<=1023&&o.legs.servo_b<=1023;
    if(!o.valid){reset(i.now_ms,i.yaw_deg);o={};return o;}
    request.roll_phase=command_.roll_phase;request.height_phase=command_.height_phase;
    return o;
}
}}
