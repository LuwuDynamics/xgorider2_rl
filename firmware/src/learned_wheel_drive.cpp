#include "learned_wheel_drive.h"

namespace rider { namespace firmware {
bool LearnedWheelDrive::eligible() const {
    return status().model_valid&&status().trained&&model().action_space==policy::ActionSpace::WheelsLegs;
}
bool LearnedWheelDrive::setMode(policy::Mode mode) {
    if(mode!=policy::Mode::Off&&!eligible())return false;
    return controller_.setMode(mode);
}
bool LearnedWheelDrive::arm() {
    controller_.resetHistory();
    return setMode(policy::Mode::Active);
}
policy::Decision LearnedWheelDrive::step(const policy::State& state,policy::Clock clock) {
    if(eligible()) {
        auto decision=controller_.step(state,0,0,clock);
        if(status().mode==policy::Mode::Active&&decision.applied&&!decision.stop)return decision;
    }
    policy::Decision stopped;stopped.stop=true;return stopped;
}
}}
