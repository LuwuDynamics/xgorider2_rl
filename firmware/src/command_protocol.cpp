#include "command_protocol.h"
#include <cmath>
#include <cstring>

namespace rider2 { namespace protocol {
namespace {
int sourceIndex(Source source) {
    switch (source) { case Source::Usb:return 0; case Source::Aux:return 1; case Source::Ble:return 2; }
    return -1;
}
uint8_t readLength(uint8_t reg) {
    switch (reg) {
    case 0x01: case 0x02: case 0xb0:return 1;
    case 0xb1:return 8;
    case 0xb2:return 16;
    case 0x07:return 10;
    case 0x0b:return 16;
    case 0x62: case 0x63: case 0x64:return 4;
    case 0x65:return 24;
    case 0x66: case 0x67: case 0x68:return 2;
    default:return 0;
    }
}
void putFloat(uint8_t *out, float value) {
    uint32_t bits;std::memcpy(&bits,&value,4);
    for (unsigned i=0;i<4;++i) out[i]=uint8_t(bits>>(i*8));
}
void putAngle(uint8_t *out, float value) {
    // Firmware trunc.s then sext16. Nonfinite inputs have no valid wire angle.
    int32_t v=0;
    if(std::isfinite(value) && value>-2147483648.0f && value<2147483648.0f) v=int32_t(value);
    const uint16_t bits=uint16_t(v);
    out[0]=uint8_t(bits>>8);out[1]=uint8_t(bits);
}
}
CommandKind commandKind(const CommandEvent &e) {
    if(e.isRead())return CommandKind::Unknown;
    switch(e.reg) {
    case 0x03:case 0x04:case 0x05:case 0x06:case 0x08:case 0x09:case 0x0a:case 0x13:
    case 0x30:case 0x32:case 0x35:case 0x36:case 0x39:case 0x3e:case 0x60:case 0x61:
    case 0x69:case 0x6a:case 0x6b:case 0x6c:case 0x82:case 0xa0:case 0xa1:case 0xb0:
    case 0xb3:case 0xb4:case 0xb5:case 0xb6:case 0xb7:case 0xb8:
        return static_cast<CommandKind>(e.reg);
    default:return CommandKind::Unknown;
    }
}
const char *commandName(CommandKind kind) {
    switch(kind) {
    case CommandKind::ActionLoop:return "action_loop";
    case CommandKind::LegCalibration:return "leg_calibration";
    case CommandKind::ReservedNoop:return "reserved_noop";
    case CommandKind::UnknownSetting06:return "setting_06_no_known_consumer";
    case CommandKind::TelemetryStream:return "telemetry_stream";
    case CommandKind::PitchCalibration:return "pitch_calibration";
    case CommandKind::HeadingHold:return "heading_hold";
    case CommandKind::BleSuffix:return "ble_suffix";
    case CommandKind::Forward:return "forward";
    case CommandKind::Turn:return "turn";
    case CommandKind::Height:return "height";
    case CommandKind::Roll:return "roll";
    case CommandKind::RollWave:return "roll_wave";
    case CommandKind::Action:return "action";
    case CommandKind::PositionHoldProtection:return "position_hold_protection";
    case CommandKind::Leveling:return "leveling";
    case CommandKind::Rgb0:return "rgb0";case CommandKind::Rgb1:return "rgb1";
    case CommandKind::Rgb2:return "rgb2";case CommandKind::Rgb3:return "rgb3";
    case CommandKind::HeightWave:return "height_wave";
    case CommandKind::MaxTurn:return "max_turn";
    case CommandKind::MaxForward:return "max_forward";
    case CommandKind::PolicyMode:return "policy_mode";
    default:return "unknown";
    }
}
bool validCommand(const CommandEvent &e) {
    if(e.type==2)return e.length==1 && readLength(e.reg)!=0;
    if(e.type>1 || commandKind(e)==CommandKind::Unknown)return false;
    if(e.reg==0x13)return e.length>=1 && e.length<=20;
    if(e.reg>=0x69 && e.reg<=0x6c)return e.length==3 || e.length==12;
    if(e.reg==0x05)return e.length<=1;
    if(e.reg==0xb0)return e.length==1&&e.data[0]<=2;
    return e.length==1;
}
void StreamParser::reset(Source source) {
    const int index=sourceIndex(source);if(index>=0)slots_[index]=Slot{};
}
bool StreamParser::receiving(Source source) const {
    const int index=sourceIndex(source);return index>=0 && slots_[index].used>0;
}
void StreamParser::expire(Source source,uint32_t now) {
    const int index=sourceIndex(source);if(index<0)return;
    Slot &s=slots_[index];
    if(s.used && uint32_t(now-s.last_ms)>100){s=Slot{};++stats_.timeouts;}
}
bool StreamParser::feed(Source source,uint8_t byte,uint32_t now,CommandEvent &event) {
    const int index=sourceIndex(source);if(index<0)return false;
    expire(source,now);
    Slot &s=slots_[index];
    s.last_ms=now;
    if(s.used==0) {if(byte==0x55)s.bytes[s.used++]=byte;return false;}
    if(s.used==1) {
        if(byte==0)s.bytes[s.used++]=byte;
        else {s.used=byte==0x55?1:0;}
        return false;
    }
    if(s.used==2) {
        if(byte<8 || byte>sizeof s.bytes) {s.used=0;s.expected=0;++stats_.bad_length;return false;}
        s.expected=byte;s.bytes[s.used++]=byte;return false;
    }
    s.bytes[s.used++]=byte;
    if(s.used<s.expected)return false;
    const uint8_t length=s.expected;
    s.used=0;s.expected=0;
    if(s.bytes[length-2]!=0 || s.bytes[length-1]!=0xaa) {++stats_.bad_trailer;if(byte==0x55)s.used=1;return false;}
    uint8_t sum=0;for(uint8_t i=2;i<length-3;++i)sum=uint8_t(sum+s.bytes[i]);
    if(uint8_t(~sum)!=s.bytes[length-3]) {++stats_.bad_checksum;return false;}
    CommandEvent candidate;candidate.source=source;candidate.type=s.bytes[3];candidate.reg=s.bytes[4];
    candidate.length=length-8;
    std::memcpy(candidate.data,s.bytes+5,candidate.length);
    if(!validCommand(candidate)) {++stats_.unsupported;return false;}
    event=candidate;++stats_.frames;return true;
}
size_t buildFrame(uint8_t type,uint8_t reg,const uint8_t *payload,size_t length,uint8_t *out,size_t cap) {
    if(!out || (length && !payload) || length>247 || cap<length+8)return 0;
    out[0]=0x55;out[1]=0;out[2]=uint8_t(length+8);out[3]=type;out[4]=reg;
    if(length)std::memcpy(out+5,payload,length);
    uint8_t sum=uint8_t(out[2]+type+reg);
    for(size_t i=0;i<length;++i)sum=uint8_t(sum+payload[i]);
    out[length+5]=uint8_t(~sum);out[length+6]=0;out[length+7]=0xaa;
    return length+8;
}
size_t buildReadResponse(const CommandEvent &r,const Telemetry &t,uint8_t *out,size_t cap) {
    if(!r.isRead() || !validCommand(r))return 0;
    uint8_t data[24]={};const uint8_t n=readLength(r.reg);
    switch(r.reg) {
    case 0x01:data[0]=std::isfinite(t.battery_percent)?uint8_t(t.battery_percent<0?0:t.battery_percent>255?255:t.battery_percent):0;break;
    case 0x02:data[0]=t.mode;break;
    case 0xb0:data[0]=t.policy_mode;break;
    case 0xb1:
        for(unsigned i=0;i<4;++i)data[i]=uint8_t(t.policy_model_id>>(8*i));
        data[4]=t.policy_action_space;data[5]=t.policy_trained;data[6]=t.policy_valid;data[7]=t.policy_contract;break;
    case 0xb2:
        data[0]=t.controller_kind;data[1]=t.wheel_source;data[2]=t.policy_mode;data[3]=t.last_stop_reason;
        for(unsigned i=0;i<4;++i){
            data[4+i]=uint8_t(t.policy_calls>>(8*i));data[8+i]=uint8_t(t.policy_failures>>(8*i));
            data[12+i]=uint8_t(t.rl_output_ticks>>(8*i));
        }break;
    case 0x07:std::memcpy(data,t.version,10);break;
    case 0x0b:for(unsigned i=0;i<4;++i)putFloat(data+4*i,t.odometry[i]);break;
    case 0x62:putFloat(data,t.roll_deg);break;
    case 0x63:putFloat(data,t.pitch_deg);break;
    case 0x64:putFloat(data,t.yaw_deg);break;
    case 0x65:
        for(unsigned i=0;i<3;++i)putFloat(data+4*i,t.accel[i]);
        putFloat(data+12,t.roll_deg);putFloat(data+16,t.pitch_deg);putFloat(data+20,t.yaw_deg);break;
    case 0x66:putAngle(data,t.roll_deg);break;
    case 0x67:putAngle(data,t.pitch_deg);break;
    case 0x68:putAngle(data,t.yaw_deg);break;
    default:return 0;
    }
    return buildFrame(0x12,r.reg,data,n,out,cap);
}
size_t buildStreamTelemetry(const Telemetry &t,uint8_t *out,size_t cap) {
    if(!out || cap<45)return 0;
    out[0]=0x55;out[1]=0;
    out[2]=std::isfinite(t.battery_percent)?uint8_t(t.battery_percent<0?0:t.battery_percent>255?255:t.battery_percent):0;
    // Preserve original double operations then final float rounding.
    const int32_t delta12=int32_t(uint32_t(t.leg12_position)-uint32_t(t.leg12_zero));
    const int32_t delta22=int32_t(uint32_t(t.leg22_position)-uint32_t(t.leg22_zero));
    putFloat(out+3,float((double(delta12)*0.0009765625)*300.0));
    putFloat(out+7,float((double(delta22)*0.0009765625)*300.0));
    putFloat(out+11,t.roll_deg);putFloat(out+15,t.pitch_deg);putFloat(out+19,t.yaw_deg);
    for(unsigned i=0;i<3;++i)putFloat(out+23+i*4,t.accel[i]);
    putFloat(out+35,t.odometry[0]);putFloat(out+39,t.odometry[1]);
    out[43]=0;out[44]=0xaa;return 45;
}
} }
