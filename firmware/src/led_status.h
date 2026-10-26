#pragma once
// Which "servos are released" indication the LEDs show (board_support.h, renderFaultLeds): from whether the robot is balancing and why it last stopped.
// Header-only so tests/test_led_status.cpp can check the mapping on the host.
#include "board_support.h"
#include "runtime_guard.h"

namespace rider { namespace firmware {

inline board::LedFault ledFaultFor(bool driving, StopReason last_stop) {
    using F = board::LedFault;
    if (driving) return F::None;                        // balancing: the servos hold, the normal colours
    switch (last_stop) {
        case StopReason::Tilt: return F::Tilt;          // fell over
        case StopReason::Fall: return F::Fall;          // free fall (stopped before the impact)
        case StopReason::Imu: case StopReason::Actuators: case StopReason::Timing: case StopReason::InvalidControl:
        case StopReason::Policy: case StopReason::InvalidCalibration: return F::Other;     // a fault: the servos were released
        case StopReason::Manual: return F::Manual;      // stopped by a command (phone, USB, switch)
        default: return F::None;                        // Startup (just powered), LowBattery (has its own blink), Calibration
    }
}

}}  // namespace rider::firmware
