#include "control/rider_leg_imu.h"
#include "leg_position_controller.h"
#include "actuator_bus.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
struct Bridge {
    rider::control::ImuState imu;
    // Mirrors the RL-only firmware: leg geometry and leveling come from
    // LegPositionController, and the wheels are owned by the actor.
    rider::firmware::LegPositionController controller;
    rider::firmware::MotionRequest request;
    rider::firmware::MotionResult motion;
    rider::bringup::ActuatorDecoder bus;
};
void rawWord(uint8_t* p,double value) {
    int v=int(std::lround(std::max(-32767.0,std::min(32767.0,value))));
    p[0]=uint8_t(uint16_t(v)>>8);p[1]=uint8_t(v);
}
}
extern "C" {
void* rider_new(){return new Bridge;}
void rider_free(void* pointer){delete static_cast<Bridge*>(pointer);}
void rider_reset(void* pointer,float pitch_deg,float roll_deg) {
    auto& b=*static_cast<Bridge*>(pointer);b=Bridge{};
    b.imu.calibrated=true;b.imu.calibration_count=251;
    b.imu.pitch_deg=b.imu.accel_pitch_filtered=pitch_deg;
    b.imu.roll_deg=b.imu.accel_roll_filtered=roll_deg;
    b.controller.reset(5000,0);b.request.leveling=true;
}
void rider_sense(void* pointer,const double* accel,const double* gyro,uint32_t us) {
    auto& b=*static_cast<Bridge*>(pointer);uint8_t raw[12];
    const double as=32768.0/19.6,gs=32768.0/2000.0;
    rawWord(raw,-accel[0]*as);rawWord(raw+2,-accel[1]*as);rawWord(raw+4,accel[2]*as);
    rawWord(raw+6,gyro[1]*gs);rawWord(raw+8,-gyro[0]*gs);rawWord(raw+10,gyro[2]*gs);
    rider::control::updateImu(b.imu,raw,12,us,5000+us/1000,1);
}
void rider_feedback(void* pointer,uint8_t id,int position,int speed,uint32_t us,float* out) {
    auto& b=*static_cast<Bridge*>(pointer);
    uint8_t packet[12]={255,255,id,8,0,uint8_t(position),uint8_t(position>>8),
        uint8_t(speed),uint8_t(uint16_t(speed)>>8),0,0,0};
    unsigned checksum=0;for(unsigned i=2;i<11;++i)checksum+=packet[i];
    packet[11]=uint8_t(~checksum);b.bus.expect(id);
    for(auto byte:packet)b.bus.feed(byte,5000+us/1000);
    const auto& left=b.bus.sample(11);const auto& right=b.bus.sample(21);
    out[0]=left.filtered_speed_mm_s;out[1]=right.filtered_speed_mm_s;
    out[2]=float(left.encoder_ticks);out[3]=float(right.encoder_ticks);
    out[4]=float(b.bus.sample(12).position_ticks);out[5]=float(b.bus.sample(22).position_ticks);
    out[6]=left.wheel_speed_mm_s;out[7]=right.wheel_speed_mm_s;
}
void rider_control(void* pointer,const float* feedback,const uint8_t* command,uint32_t us,float* out) {
    auto& b=*static_cast<Bridge*>(pointer);rider::firmware::MotionInput input;
    input.now_ms=5000+us/1000;input.mode=1;input.calibrated=true;
    input.pitch_deg=b.imu.pitch_deg;input.roll_deg=b.imu.roll_deg;input.yaw_deg=b.imu.yaw_deg;
    input.pitch_rate_rad_s=b.imu.pitch_rate_rad_s;
    input.speed11=feedback[0];input.speed21=feedback[1];input.encoder11=int32_t(feedback[2]);input.encoder21=int32_t(feedback[3]);
    input.leg12_position=int16_t(feedback[4]);input.zero_a=610;input.zero_b=431;
    b.request.command.forward=command[0];b.request.command.turn=command[1];b.request.command.height=command[2];
    // The trained profile assumes the roll-leveling leg loop, as in the
    // RL-only firmware; the app commands still drive height and motion.
    b.request.leveling=true;
    b.motion=b.controller.step(input,b.request);
    out[0]=b.imu.pitch_deg;out[1]=b.imu.roll_deg;out[2]=b.imu.yaw_deg;
    out[3]=b.imu.pitch_rate_rad_s;out[4]=b.imu.gyro_roll_filtered*.017453292519943295f;
    out[5]=(b.imu.gyro_deg_s[2]-b.imu.gyro_bias[2])*.017453292519943295f;
    out[6]=feedback[6]*.001f;out[7]=feedback[7]*.001f;
    out[8]=(feedback[4]-610)/512;out[9]=(feedback[5]-431)/512;
    out[10]=b.motion.command.filtered_forward*.001f;out[11]=b.motion.command.filtered_turn*.017453292519943295f;
    out[12]=b.motion.command.filtered_height/36;out[13]=b.motion.command.roll/20;
    // No legacy wheel output exists in the RL-only controller.
    out[14]=0;out[15]=0;
    const double scale=6.283185307179586/1024;
    out[16]=float(.096*std::cos(1.1408376963350786-(610-int(b.motion.legs.servo_a))*scale));
    out[17]=float(.096*std::cos(1.1408376963350786-(int(b.motion.legs.servo_b)-431)*scale));
}
}
