#include "action_engine.h"
#include <cmath>

namespace rider2 {
namespace { constexpr uint8_t sequence[6]={1,2,3,4,5,6}; }

ActionEngine::ActionEngine() = default;

void ActionEngine::event(ActionEvent flag, uint8_t id) {
    events_.flags|=uint32_t(flag); events_.action_id=id; events_.mode=state_.mode;
}
ActionEvents ActionEngine::takeEvents() {
    const ActionEvents result=events_; events_={}; events_.mode=state_.mode; return result;
}
void ActionEngine::clearCounters() {
    // 0x400d4728: the Xtensa LOOP instruction clears all 255 uint16 entries.
    for(auto &value:counters_) value=0;
}
void ActionEngine::resetCommands(uint32_t now_ms) {
    // 0x400d4740. Limits, periods, heading hold and automatic lean are retained.
    state_.last_reset_ms=now_ms;
    state_.leveling=false; state_.hold_protection=true;
    state_.roll_phase=0; state_.height_phase=0;
    state_.command.forward=128; state_.command.turn=128;
    state_.command.roll=128; state_.command.height=144;
    event(CommandsReset); event(HoldReset);
}
void ActionEngine::finish(uint8_t kind, uint32_t now_ms) {
    // 0x400d4788. Kind 0 initializes a preset, 1 completes it, 2 cancels looping.
    const uint8_t completed_id=state_.action_id;
    resetCommands(now_ms); clearCounters();
    if(kind==1) {
        if(state_.loop) state_.loop_wait_ticks=1;
        state_.action_id=0; event(ActionFinished,completed_id);
    } else if(kind==2) {
        resetCommands(now_ms); // This duplicate reset is present in the binary.
        state_.loop=0; state_.action_id=0; state_.sequence_index=1;
    }
}
void ActionEngine::cancel(uint32_t now_ms) {
    const uint8_t cancelled_id=state_.action_id;
    restore_pending_=false; finish(2,now_ms); event(ActionCancelled,cancelled_id);
}
bool ActionEngine::requestMode(uint8_t mode, bool force) {
    if(!force && (state_.mode&0x80)) return false;
    if(state_.mode!=mode) {state_.mode=mode; event(ModeChanged,state_.action_id);}
    return true;
}
bool ActionEngine::enable(uint32_t now_ms) {
    // Note: mode 250 meets the <251 test but its bit7 blocks the mode=1 write.
    if(state_.mode>=251) return false;
    const bool accepted=requestMode(1,false);
    restore_pending_=false; finish(2,now_ms);
    return accepted;
}
void ActionEngine::setLoop(uint8_t value, uint32_t now_ms) {
    state_.loop=value;
    if(value) state_.action_id=1;
    else cancel(now_ms);
}
bool ActionEngine::request(uint8_t id, uint32_t now_ms) {
    // A new externally supplied command cancels the nonblocking tail delay.
    // This makes cancellation prompt; normal uninterrupted action timing is exact.
    restore_pending_=false; state_.action_id=id; tick(now_ms); return supported(id);
}
void ActionEngine::stopCommands() {
    // 0x400d47f0. The runtime consumes the reset event to zero wheel outputs and
    // speed/pitch PID integral/previous-error values. Forward/turn bytes survive.
    state_.roll_phase=0; state_.height_phase=0; state_.leveling=false;
    state_.command.height=0; state_.command.roll=128;
    event(SpeedPitchPidReset,state_.action_id);
}
void ActionEngine::poll(uint32_t now_ms) {
    if(restore_pending_ && uint32_t(now_ms-restore_started_ms_)>=50) {
        restore_pending_=false; state_.heading_hold=saved_heading_hold_;
        ++counters_[5]; event(DeferredRestore,5);
    }
}
void ActionEngine::tick(uint32_t now_ms) {
    poll(now_ms);
    if(restore_pending_) return;
    if(state_.loop_wait_ticks && state_.loop) {
        state_.loop_wait_ticks=uint8_t(state_.loop_wait_ticks+1);
        if(state_.loop_wait_ticks==30) {
            state_.loop_wait_ticks=0;
            // Normal firmware index is 0..5; guard malformed host state.
            if(state_.sequence_index>5) state_.sequence_index=0;
            state_.action_id=sequence[state_.sequence_index];
            state_.sequence_index=state_.sequence_index==5?0:uint8_t(state_.sequence_index+1);
        }
    }
    const uint8_t id=state_.action_id;
    if(id==0) {clearCounters(); return;}
    if(id==255) {enable(now_ms); return;}
    if(id>6) {state_.action_id=0; event(ActionUnknown,id); return;}
    const uint16_t step=counters_[id];
    if(id==5) state_.heading_hold=false; // BEFORE saving, as in 0x400d49b0.
    if(step==0) {
        finish(0,now_ms); event(ActionStarted,id);
        switch(id) {
        case 1: state_.command.height=112; state_.roll_phase=10; break;
        case 2: state_.height_phase=10; break;
        case 3: state_.command.forward=192; break;
        case 4: state_.command.forward=176; state_.command.turn=208; break;
        case 5:
            state_.height_phase=10; state_.command.turn=32;
            saved_heading_hold_=state_.heading_hold; break;
        case 6:
            state_.command.roll_period=128; state_.command.height_period=128;
            state_.roll_phase=10; state_.height_phase=750; break;
        }
    } else {
        switch(id) {
        case 1: if(step==50) finish(1,now_ms); break;
        case 2: if(step==40) finish(1,now_ms); break;
        case 3:
            if(step==8) state_.command.forward=64;
            else if(step==20) finish(1,now_ms);
            break;
        case 4:
            if(step==14) {state_.command.forward=176; state_.command.turn=48;}
            else if(step==28) {state_.command.forward=96; state_.command.turn=208;}
            else if(step==44) {state_.command.forward=96; state_.command.turn=48;}
            else if(step==60) finish(1,now_ms);
            break;
        case 5:
            if(step==46) state_.command.turn=208;
            else if(step==100) {
                finish(1,now_ms); restore_pending_=true; restore_started_ms_=now_ms;
                return; // Original increment occurs after delay(50), not before.
            }
            break;
        case 6: if(step==80) finish(1,now_ms); break;
        }
    }
    // Read-modify-write AFTER finish(), whose clearCounters() may have run.
    ++counters_[id];
}
bool ActionEngine::controlTick(uint32_t now_ms, float pitch_deg, float roll_deg) {
    poll(now_ms);
    if(state_.mode==1) return false;
    state_.last_reset_ms=now_ms; stopCommands();
    if(state_.mode>249 || !state_.auto_recover) return false;
    if(std::fabs(pitch_deg)<8 && std::fabs(roll_deg)<5) {
        ++state_.upright_control_ticks;
        if(state_.upright_control_ticks>500) {
            state_.upright_control_ticks=0;
            const bool enabled=enable(now_ms);
            if(enabled) event(AutoRecovered);
            return enabled;
        }
    } else state_.upright_control_ticks=0;
    return false;
}
} // namespace rider2
