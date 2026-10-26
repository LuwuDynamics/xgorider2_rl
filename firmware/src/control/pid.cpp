#include "pid.h"

namespace rider { namespace control {
namespace {
float clampSymmetric(float value,float limit) {
    if(value<=-limit)return -limit;
    if(value>=limit)return limit;
    return value;
}
Pid makePid(float kp,float ki,float kd,float output_limit,float integral_limit,
            float deadband,float p_limit,float i_limit,float d_limit) {
    Pid p;
    p.kp=kp;p.ki=ki;p.kd=kd;
    p.output_limit=output_limit;p.integral_limit=integral_limit;p.deadband=deadband;
    p.p_limit=p_limit;p.i_limit=i_limit;p.d_limit=d_limit;
    return p;
}
}

float updatePid(Pid &p) {
    // 0x400d3850..0x400d3971. Gains are per call; the binary has no dt term.
    p.error=p.target-p.measurement;
    if(p.error*p.previous_error<0)p.integral=0;
    p.integral=clampSymmetric(p.error+p.integral,p.integral_limit);
    p.i_term=clampSymmetric(p.integral*p.ki,p.i_limit);
    p.d_term=clampSymmetric((p.error-p.previous_error)*p.kd,p.d_limit);
    p.p_term=clampSymmetric(p.error*p.kp,p.p_limit);
    if(p.error<p.deadband && p.error>-p.deadband)
        p.integral=static_cast<float>(static_cast<double>(p.integral)*0.95);
    // Decay changes next call's integral. This call's i_term was already formed.
    p.previous_error=p.error;
    p.output=clampSymmetric((p.i_term+p.p_term)+p.d_term,p.output_limit);
    return p.output;
}

Pid headingPid() {
    // Initial state 0x3ffbdf8c; 0x400d4b9e/4ba1/4ba4 set term limits
    // from 0x3ffbdf6c (float32 300). They are zero in the image's data template.
    return makePid(3,0,.5f,1023,3000,2,300,300,300);
}
Pid rollPid() {return makePid(.02f,0,.006f,1,3000,.1f,.8f,.8f,.4f);}
Pid pitchPid() {
    // Data template has kp=70,kd=52. 0x400d5868/586d overwrite these each loop.
    return makePid(37.5f,0,0,1023,1023,.2f,1023,1023,20000);
}
Pid speedPid(bool profile_two) {
    // 0x400d5855..5860 / 0x400d58cc..58d4, selected by byte 0x3ffc30a8==2.
    return makePid(profile_two?.14f:.11f,
                   profile_two?.000736842107f:.000733333349f,
                   0,100,100,0,100,100,100);
}

float mapByte(std::uint8_t value,std::uint8_t input_min,std::uint8_t input_max,
              float output_min,float output_max) {
    // 0x400d3974. Equal input endpoints preserve the binary's 0/0 behavior.
    if(value<input_min)return output_min;
    if(value>input_max)return output_max;
    const float numerator=static_cast<float>(value-input_min)*(output_max-output_min);
    return output_min+numerator/static_cast<float>(input_max-input_min);
}
float applyDeadband(float value,float threshold,std::uint8_t mode) {
    // 0x400d39d0. At zero, nonzero mode chooses positive threshold.
    if(value>-threshold && value<threshold)
        return mode==0?0.0f:(value<0?-threshold:threshold);
    return value;
}

}} // namespace rider::control
