#include "actuator_bus.h"
#include <initializer_list>
#ifdef ARDUINO
#include <Arduino.h>
#include <HardwareSerial.h>
#endif

namespace rider { namespace bringup {
namespace {
constexpr std::uint8_t kIds[4] = {11, 21, 12, 22};
int indexOf(std::uint8_t id) {
    for (int i = 0; i < 4; ++i) if (kIds[i] == id) return i;
    return -1;
}
std::int16_t signedWord(const std::uint8_t *p) {
    // 0x40198028: sign-extend high byte, shift 8, OR low, sign-extend 16.
    const std::uint16_t value = std::uint16_t(p[0]) | (std::uint16_t(p[1]) << 8);
    return value < 0x8000u ? std::int16_t(value) : std::int16_t(std::int32_t(value) - 65536);
}
std::int32_t addWrap32(std::int32_t a, std::int32_t b) {
    const auto u = std::uint32_t(a) + std::uint32_t(b);
    return u < 0x80000000u ? std::int32_t(u) : std::int32_t(std::int64_t(u) - 4294967296LL);
}
int limit(int value, int lo, int hi) { return value<lo?lo:value>hi?hi:value; }
void finish(ActuatorPacket &p) {
    std::uint8_t sum=0;
    for(unsigned i=2;i+1<p.size;++i)sum=std::uint8_t(sum+p.bytes[i]);
    p.bytes[p.size-1]=std::uint8_t(~sum);
}
ActuatorPacket syncPacket(const std::uint8_t *ids,const int *pos,const int *speed,unsigned n) {
    ActuatorPacket p;
    if(n==0||n>4)return p;
    p.size=std::uint8_t(n*5+8);
    p.bytes[0]=255;p.bytes[1]=255;p.bytes[2]=254;p.bytes[3]=std::uint8_t(n*5+4);
    p.bytes[4]=0x83;p.bytes[5]=0x1e;p.bytes[6]=4;
    for(unsigned i=0;i<n;++i) {
        const bool wheel=ids[i]==11||ids[i]==21;
        const int position=wheel?0:limit(pos[i],0,1023);
        const int velocity=wheel?limit(speed[i],-1023,1023):0;
        const unsigned offset=7+i*5;
        p.bytes[offset]=ids[i];p.bytes[offset+1]=std::uint8_t(position);
        p.bytes[offset+2]=std::uint8_t(unsigned(position)>>8);
        const auto word=std::uint16_t(velocity);
        p.bytes[offset+3]=std::uint8_t(word);p.bytes[offset+4]=std::uint8_t(word>>8);
    }
    finish(p);return p;
}
ActuatorPacket writeByte(std::uint8_t id,std::uint8_t address,std::uint8_t value) {
    ActuatorPacket p;
    if(indexOf(id)<0)return p;
    p.size=8;p.bytes[0]=255;p.bytes[1]=255;p.bytes[2]=id;p.bytes[3]=4;
    p.bytes[4]=3;p.bytes[5]=address;p.bytes[6]=value;finish(p);return p;
}
}

ActuatorPacket makeActuatorOutput(int w11,int w21,int leg12,int leg22,bool legs) {
    const int positions[]={0,0,leg12,leg22},speeds[]={w11,w21,0,0};
    return syncPacket(kIds,positions,speeds,legs?4:2);
}
ActuatorPacket makeSingleWheelSpeed(std::uint8_t id,int speed) {
    if(id!=11&&id!=21)return {};
    const int position=0;return syncPacket(&id,&position,&speed,1);
}
ActuatorPacket makeSingleLegPosition(std::uint8_t id,int position) {
    if(id!=12&&id!=22)return {};
    const int speed=0;return syncPacket(&id,&position,&speed,1);
}
ActuatorPacket makeServoTorque(std::uint8_t id,bool enabled) { return writeByte(id,0x18,enabled?1:0); }
ActuatorPacket makeWheelReturnLevel(std::uint8_t id) {
    if(id!=11&&id!=21)return {};
    return writeByte(id,0x11,0);
}
bool ActuatorCommandQueue::push(const ActuatorPacket &p) {
    if(!p.size)return false;
    if(count_==16){++overflows_;return false;}
    if(!urgent_)stopped_=false;
    queue_[(head_+count_)%16]=p;++count_;return true;
}
void ActuatorCommandQueue::setOutputs(int a,int b,int c,int d,bool legs) {
    if(urgent_)return; // an unfinished stop cannot replay a later motion mailbox
    stopped_=false;
    output_=makeActuatorOutput(a,b,c,d,legs);output_pending_=true;
}
bool ActuatorCommandQueue::setLegTorque(bool enabled) {
    if(count_>12){++overflows_;return false;}
    for(auto id:{22,12,22,12})push(makeServoTorque(std::uint8_t(id),enabled));
    return true;
}
void ActuatorCommandQueue::stop(bool release_legs) {
    output_pending_=false;
    // A repeated request is idempotent, but stop(false)->stop(true) must add
    // the stronger leg-release sequence even if the wheel-only stop completed.
    if((urgent_||stopped_)&&(!release_legs||stop_releases_))return;
    head_=0;count_=0;output_pending_=false;urgent_=true;
    stop_releases_=release_legs;
    push(makeActuatorOutput(0,0,0,0,false));
    if(release_legs)setLegTorque(false);
    stopped_=true;
}
bool ActuatorCommandQueue::next(ActuatorPacket &out) {
    if(count_){out=queue_[head_];head_=std::uint8_t((head_+1)%16);--count_;if(!count_)urgent_=false;return true;}
    if(output_pending_){out=output_;output_pending_=false;return true;}
    return false;
}

std::size_t makeActuatorReadRequest(std::uint8_t id, std::uint8_t out[8]) {
    if (!out || indexOf(id) < 0) return 0;
    // Original 0x400d4d70. 0x02 is READ, 0x24 is present position.
    out[0] = 0xff; out[1] = 0xff; out[2] = id; out[3] = 4;
    out[4] = 2; out[5] = 0x24; out[6] = 6;
    out[7] = std::uint8_t(~std::uint8_t(id + 0x30));
    return 8;
}

ActuatorDecoder::ActuatorDecoder() {
    for (int i = 0; i < 4; ++i) samples_[i].id = kIds[i];
}
void ActuatorDecoder::resetFrame() { used_ = 0; }
void ActuatorDecoder::expect(std::uint8_t id) {
    expected_id_ = indexOf(id) >= 0 ? id : 0;
    resetFrame();
}
void ActuatorDecoder::clearExpectation() { expected_id_ = 0; resetFrame(); }
const ActuatorSample &ActuatorDecoder::sample(std::uint8_t id) const {
    const int index = indexOf(id);
    return index >= 0 ? samples_[index] : unknown_;
}
void ActuatorDecoder::expirePartial(std::uint32_t now_ms) {
    if (used_ && std::uint32_t(now_ms - last_byte_ms_) > 3) {
        ++stats_.partial_timeouts;
        resetFrame();
    }
}
bool ActuatorDecoder::feed(std::uint8_t byte, std::uint32_t now_ms) {
    ++stats_.rx_bytes;
    expirePartial(now_ms);
    last_byte_ms_ = now_ms;
    if (used_ < 2) {
        if (byte == 0xff) bytes_[used_++] = byte;
        else { ++stats_.noise_bytes; resetFrame(); }
        return false;
    }
    if (used_ == 2) {
        // More than two FF bytes still form a usable header.
        if (byte == 0xff) { ++stats_.noise_bytes; return false; }
        bytes_[used_++] = byte;
        return false;
    }
    if (used_ == 3 && byte != 8 && byte != 11 && byte != 2) {
        ++stats_.length_errors;
        resetFrame();
        if (byte == 0xff) bytes_[used_++] = byte;
        return false;
    }
    bytes_[used_++] = byte;
    if (used_ != std::uint8_t(bytes_[3]+4)) return false;
    resetFrame();
    return decode(now_ms);
}
bool ActuatorDecoder::decode(std::uint32_t now_ms) {
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < std::size_t(bytes_[3]+4); ++i) sum = std::uint8_t(sum + bytes_[i]);
    if (sum != 0xff) { ++stats_.checksum_errors; return false; }
    const int index = indexOf(bytes_[2]);
    if (index < 0) { ++stats_.id_errors; return false; }
    if(bytes_[3]==2) {
        ++stats_.ack_frames;
        samples_[index].last_status=bytes_[4];samples_[index].last_status_ms=now_ms;
        if(bytes_[4]) {++stats_.status_errors;if(bytes_[2]==expected_id_)expected_id_=0;}
        return false;
    }
    if (bytes_[2] != expected_id_) { ++stats_.unexpected_frames; return false; }
    auto &s = samples_[index];
    s.last_status = bytes_[4];
    s.last_status_ms = now_ms;
    if (bytes_[4]) { ++stats_.status_errors; expected_id_ = 0; return false; }
    // Original 0x400d5be0..0x400d5c14 reads the final six data bytes for
    // both accepted feedback lengths, irrespective of the three extra prefix
    // bytes in LENGTH=11. These are offsets 5/7/9 or 8/10/12 respectively.
    const auto *feedback = bytes_ + bytes_[3] - 3;
    const std::int16_t position = signedWord(feedback);
    // Rider's joints and wheels use 1024 ticks/revolution, unlike Ranger joints.
    if (position < 0 || position > 1023) {
        ++stats_.position_errors; expected_id_ = 0; return false;
    }
    const std::int16_t speed = signedWord(feedback + 2);
    const bool is_wheel = s.id == 11 || s.id == 21;
    if (is_wheel) {
        // 0x400d5a30 keeps the unusual 1023 wrap and strict +/-800 threshold.
        std::int32_t delta = std::int32_t(position) - s.position_ticks;
        if (delta < -800) delta += 1023;
        else if (delta > 800) delta -= 1023;
        if (s.id == 21) delta = -delta;
        s.encoder_ticks = addWrap32(s.encoder_ticks, delta);
        if (s.valid) {
            s.relative_encoder_ticks = addWrap32(s.relative_encoder_ticks, delta);
            if (std::uint32_t(now_ms - s.last_update_ms) > 100) s.encoder_continuity_lost = true;
        }
        // ID21 negates a signed16 temporary in the original firmware, including
        // the -32768 edge case. Keep that narrowing explicit and deterministic.
        std::int32_t oriented_speed = speed;
        if (s.id == 21) oriented_speed = speed == -32768 ? -32768 : -std::int32_t(speed);
        const float scaled = float(oriented_speed) * 4.8f;
        s.wheel_speed_mm_s = float(((double(scaled) * 55.0) * 3.1413) * 0.0009765625);
        s.filtered_speed_mm_s = float(double(s.filtered_speed_mm_s) * 0.75 +
                                      double(s.wheel_speed_mm_s) * 0.25);
    }
    s.position_ticks = position;
    s.raw_speed = speed;
    s.load_raw = signedWord(feedback + 4);
    s.last_update_ms = now_ms;
    ++s.sample_count;
    s.valid = true;
    ++stats_.valid_frames;
    if (bytes_[3] == 11) ++stats_.valid_extended_frames;
    expected_id_ = 0;
    return true;
}

#ifdef ARDUINO
void ActuatorBus::begin() {
    // UART object mapping 0x400dcc78, UART begin call 0x400d4bbc.
    serial_ = &Serial2;
    serial_->setRxBufferSize(256);
    serial_->begin(kActuatorBaud, SERIAL_8N1, kActuatorRxPin, kActuatorTxPin);
    for (unsigned i = 0; i < 256 && serial_->available(); ++i) serial_->read();
    decoder_ = ActuatorDecoder{};
    commands_ = ActuatorCommandQueue{};
    setup_index_=0;retry_pending_=false;
    schedule_index_ = 0;
    last_was_read_=false;feedback_paused_=false;
    last_request_ms_ = last_tx_ms_ = millis();
    started_ = true;
}
void ActuatorBus::poll(std::uint32_t now_ms) {
    if (!started_) return;
    decoder_.expirePartial(now_ms);
    for (unsigned i = 0; i < 256 && serial_->available(); ++i) {
        const int value = serial_->read();
        if (value >= 0) decoder_.feed(std::uint8_t(value), now_ms);
    }
    if (decoder_.expectedId() && std::uint32_t(now_ms - last_request_ms_) >= kActuatorResponseTimeoutMs) {
        ++decoder_.stats_.response_timeouts;
        decoder_.clearExpectation();
    }
    decoder_.stats_.queue_overflows=commands_.overflows();
    // A read owns the bus until complete/timeout. Each TX fully drains before
    // its guard interval; FIFO and latest-output mailbox share this sole writer.
    if (decoder_.expectedId() || std::uint32_t(now_ms-last_tx_ms_)<1) return;
    if(retry_pending_) {
        if(serial_->write(retry_packet_.bytes,retry_packet_.size)==retry_packet_.size) {
            ++decoder_.stats_.writes_sent;retry_pending_=false;
        } else ++decoder_.stats_.tx_errors;
        serial_->flush();last_tx_ms_=now_ms;last_was_read_=false;return;
    }
    if(setup_index_<2&&!commands_.urgent()) {
        const auto packet=makeWheelReturnLevel(setup_index_?21:11);
        if(serial_->write(packet.bytes,packet.size)==packet.size){++decoder_.stats_.writes_sent;++setup_index_;}
        else ++decoder_.stats_.tx_errors;
        serial_->flush();last_tx_ms_=now_ms;last_was_read_=false;return;
    }
    if(commands_.hasPending()&&(feedback_paused_||commands_.urgent()||last_was_read_||
                               std::uint32_t(now_ms-last_request_ms_)<kActuatorRequestIntervalMs)) {
        ActuatorPacket packet;
        const bool safe_retry=commands_.urgent();
        if(commands_.next(packet)) {
            if(serial_->write(packet.bytes,packet.size)==packet.size)++decoder_.stats_.writes_sent;
            else {
                ++decoder_.stats_.tx_errors;retry_packet_=packet;
                retry_pending_=true;retry_safe_=safe_retry;
            }
            serial_->flush();last_tx_ms_=now_ms;last_was_read_=false;
        }
        return;
    }
    if(feedback_paused_||std::uint32_t(now_ms-last_request_ms_)<kActuatorRequestIntervalMs)return;
    // Original 0x400d51f8 schedule, interleaved with output frames.
    constexpr std::uint8_t schedule[10] = {11, 21, 11, 21, 11, 21, 11, 21, 12, 22};
    const std::uint8_t id = schedule[schedule_index_];
    schedule_index_ = std::uint8_t((schedule_index_ + 1) % 10);
    std::uint8_t request[8];
    const auto count = makeActuatorReadRequest(id, request);
    decoder_.expect(id);
    if (count == 8 && serial_->write(request, count) == count) ++decoder_.stats_.requests_sent;
    else { ++decoder_.stats_.tx_errors; decoder_.clearExpectation(); }
    serial_->flush();
    last_request_ms_ = last_tx_ms_ = now_ms;last_was_read_=true;
}
bool ActuatorBus::transportIdle() const {
    return started_&&!writesPending()&&!decoder_.expectedId()&&std::uint32_t(millis()-last_tx_ms_)>=1;
}
#endif

}} // namespace rider::bringup
