#pragma once
#include <cstdint>
#include "control/rider_motion.h"

namespace rider2 {

enum ActionEvent : uint32_t {
    ActionNoEvent=0, ActionStarted=1u<<0, ActionFinished=1u<<1,
    ActionCancelled=1u<<2, ActionUnknown=1u<<3, CommandsReset=1u<<4,
    HoldReset=1u<<5, SpeedPitchPidReset=1u<<6, ModeChanged=1u<<7,
    AutoRecovered=1u<<8, DeferredRestore=1u<<9
};
struct ActionEvents {
    uint32_t flags=0;
    uint8_t action_id=0, mode=253;
    bool has(ActionEvent flag) const { return (flags & uint32_t(flag))!=0; }
};

// This state corresponds to the original command globals. The protocol and
// controller may update it on the SAME owner task; ActionEngine is not threaded.
struct ActionState {
    rider::control::MotionCommand command{};
    float roll_phase=0, height_phase=0;
    bool leveling=false, heading_hold=true, hold_protection=true;
    uint8_t mode=253, action_id=0, loop=0, sequence_index=1, loop_wait_ticks=0;
    uint32_t last_reset_ms=0, upright_control_ticks=0;
    bool auto_recover=true; // OEM always tests upright recovery in modes 0..249.
};

class ActionEngine {
public:
    ActionEngine();
    ActionState &state() { return state_; }
    const ActionState &state() const { return state_; }
    // Original protocol 0x3e assigns ID then IMMEDIATELY runs one action tick.
    // A repeated ID continues its counter; it does not implicitly restart it.
    bool request(uint8_t id, uint32_t now_ms);
    // Original protocol 0x03: nonzero selects action 1 without an immediate tick;
    // zero performs finish/reset kind 2, including sequence_index=1.
    void setLoop(uint8_t value, uint32_t now_ms);
    // Original 0x400d4788(2), an explicit cancellation extension to the host API.
    void cancel(uint32_t now_ms);
    void reset(uint32_t now_ms) { cancel(now_ms); }
    // Original 0x400d47d0: request mode 1 only while mode<251, then reset kind 2.
    bool enable(uint32_t now_ms);
    // Original 0x400d510c: non-forced writes cannot replace a mode with bit7 set.
    bool requestMode(uint8_t mode, bool force=false);
    // Stop command fields/PID outputs, as in 0x400d47f0; does not change mode.
    void stopCommands();
    // Exactly ONE 0x400d4a94 invocation. Caller owns its 21-sensor-loop cadence.
    // No elapsed-time catch-up is applied; action phases are invocation counts.
    void tick(uint32_t now_ms);
    // Nonblocking completion of action 5's original 50 ms delay; wrap-safe.
    // Call each runtime iteration, even when tick() is not due.
    void poll(uint32_t now_ms);
    // Original stopped-control path, including >500 upright calls recovery.
    // Return true when this invocation requests auto recovery.
    bool controlTick(uint32_t now_ms, float pitch_deg, float roll_deg);
    ActionEvents takeEvents();
    uint16_t counter(uint8_t id) const { return id<255 ? counters_[id] : 0; }
    bool deferred() const { return restore_pending_; }
    static bool supported(uint8_t id) { return id<=6 || id==255; }
private:
    void resetCommands(uint32_t now_ms);
    void clearCounters();
    void finish(uint8_t kind, uint32_t now_ms);
    void event(ActionEvent flag, uint8_t id=0);
    ActionState state_{};
    ActionEvents events_{};
    uint16_t counters_[255]{};
    bool saved_heading_hold_=false, restore_pending_=false;
    uint32_t restore_started_ms_=0;
};
} // namespace rider2
