#pragma once
#include <cmath>
#include <cstdint>
namespace rider { namespace firmware {
// Exact known 0x400d5f0c arithmetic, including its unusual encoder/speed mixture
// and passing yaw in degrees to sinf/cosf. Protocol compatibility only: do not
// present these values as validated physical localization.
struct OriginalOdometry {
    float x=0,y=0,heading=0,distance=0;
    uint32_t last_ms=0;
    void update(uint32_t now_ms,int32_t encoder21,float speed11,float yaw_deg) {
        const uint32_t dt=now_ms-last_ms;last_ms=now_ms;
        if(dt>100||!std::isfinite(speed11)||!std::isfinite(yaw_deg))return;
        const float speed=(float(encoder21)+speed11)*.5f;
        const float increment=float(dt)*speed;
        x=float(double(x)+double(increment*std::cos(yaw_deg))/1000.0);
        y=float(double(y)+double(increment*std::sin(yaw_deg))/1000.0);
        heading=yaw_deg;distance=float(double(distance)+double(increment)/1000.0);
    }
};
}}
