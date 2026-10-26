#pragma once
#include <cstdint>

// Rider R-1.1.6 binary recovery. Names are assigned from data flow, not symbols.
namespace rider { namespace control {

struct Pid {
    // Exact 72-byte state layout used by 0x400d3850.
    float target=0, measurement=0, kp=0, ki=0, kd=0;
    float error=0, previous_error=0, integral=0, output=0;
    float output_limit=0, integral_limit=0, deadband=0;
    float p_term=0, p_limit=0, i_term=0, i_limit=0, d_term=0, d_limit=0;
};
static_assert(sizeof(Pid)==72, "Recovered PID layout requires 18 float32 fields");

float updatePid(Pid &state);
// Factories reflect values after known initialization/runtime writes.
Pid headingPid();
Pid rollPid();
Pid pitchPid();
Pid speedPid(bool profile_two=false);

float mapByte(std::uint8_t value, std::uint8_t input_min,
              std::uint8_t input_max, float output_min, float output_max);
// Strict interior only. mode=0 zeroes; any nonzero mode pushes to +/-threshold.
float applyDeadband(float value, float threshold, std::uint8_t mode=0);

}} // namespace rider::control
