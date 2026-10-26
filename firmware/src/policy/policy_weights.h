#pragma once
#include "policy.h"

// Diagnostic weights only. This is NOT a trained balancing controller.
// Replace using tools/export_policy.py after simulation training/validation.
// policy_export: {"name":"diagnostic_untrained","model_id":0,"trained":false,"action_space":"residual","action_scale":80.0,"contract":6,"parameters":1796}
namespace rider { namespace policy { namespace weights {
constexpr unsigned kExportedContract=6;
inline const Model& compiledModel() {
    static const float mean[kInputs]={};
    static const float inv_std[kInputs]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    static const float w1[kHidden*kInputs]={0.125f},b1[kHidden]={};
    static const float w2[kHidden*kHidden]={0.25f},b2[kHidden]={};
    static const float w3[kOutputs*kHidden]={0.5f},b3[kOutputs]={};
    static const Model model={"diagnostic_untrained",0,6,false,ActionSpace::Residual,80,.005f,5,
        mean,inv_std,w1,b1,w2,b2,w3,b3};
    return model;
}
}}}
