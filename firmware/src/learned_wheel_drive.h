#pragma once
#include "policy/policy.h"

namespace rider { namespace firmware {
// A direct-wheel actor is the sole possible source of wheel commands. There is
// deliberately no baseline argument; Off/Shadow/fault can only request stop.
class LearnedWheelDrive {
public:
    explicit LearnedWheelDrive(const policy::Model& model):controller_(model){}
    bool arm();
    bool setMode(policy::Mode);
    void resetHistory(){controller_.resetHistory();}
    void setSlewRate(float rate){controller_.setSlewRate(rate);}
    bool setDeadZone(const policy::DeadZoneConfig& config){return controller_.setDeadZone(config);}
    const policy::DeadZoneConfig& deadZoneConfig() const{return controller_.deadZoneConfig();}
    void resetDeadZoneLearning(){controller_.resetDeadZoneLearning();}
    policy::Decision step(const policy::State&,policy::Clock);
    const policy::Status& status() const{return controller_.status();}
    const policy::Model& model() const{return controller_.model();}
private:
    bool eligible() const;
    policy::Controller controller_;
};
}}
