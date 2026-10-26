#pragma once
#include <cstddef>
#include <cstdint>

#ifdef ARDUINO
class HardwareSerial;
#endif

namespace rider { namespace bringup {

constexpr std::uint32_t kActuatorBaud = 1000000;
constexpr int kActuatorUart = 2;
constexpr int kActuatorRxPin = 13;
constexpr int kActuatorTxPin = 14;
constexpr std::uint32_t kActuatorRequestIntervalMs = 2;
constexpr std::uint32_t kActuatorResponseTimeoutMs = 4;

struct ActuatorSample {
    std::uint8_t id = 0;
    bool valid = false;
    std::uint32_t last_update_ms = 0, sample_count = 0;
    std::int16_t position_ticks = 0, raw_speed = 0, load_raw = 0;
    // Firmware signs: ID 11 positive, ID 21 negative. Millimetres/second.
    float wheel_speed_mm_s = 0, filtered_speed_mm_s = 0;
    // Firmware-compatible accumulator, including first sample vs previous=0.
    std::int32_t encoder_ticks = 0;
    // Bring-up-only relative accumulator, zeroed at the first valid sample.
    std::int32_t relative_encoder_ticks = 0;
    bool encoder_continuity_lost = false;
    std::uint8_t last_status = 0;
    std::uint32_t last_status_ms = 0;
    std::uint32_t ageMs(std::uint32_t now) const {
        return valid ? std::uint32_t(now - last_update_ms) : UINT32_MAX;
    }
    bool fresh(std::uint32_t now, std::uint32_t max_age_ms) const {
        return valid && ageMs(now) <= max_age_ms;
    }
};

struct ActuatorBusStats {
    std::uint32_t rx_bytes = 0, valid_frames = 0, noise_bytes = 0;
    std::uint32_t length_errors = 0, checksum_errors = 0, id_errors = 0;
    std::uint32_t unexpected_frames = 0, status_errors = 0, position_errors = 0;
    std::uint32_t partial_timeouts = 0, response_timeouts = 0;
    std::uint32_t requests_sent = 0, tx_errors = 0;
    std::uint32_t writes_sent = 0, queue_overflows = 0;
    std::uint32_t ack_frames = 0;
    std::uint32_t valid_extended_frames = 0;
};

// Produces only a six-register READ at address 0x24 for IDs 11/21/12/22.
// The read request is retained alongside the recovered output commands below.
std::size_t makeActuatorReadRequest(std::uint8_t id, std::uint8_t out[8]);

struct ActuatorPacket { std::uint8_t bytes[28] = {}; std::uint8_t size = 0; };
// Recovered sync-write: FF FF FE (5*N+4) 83 1E 04 [ID POS_LE SPEED_LE] CHECKSUM.
// Wheels get position zero and signed16 speed; legs get position and speed zero.
ActuatorPacket makeActuatorOutput(int wheel11, int wheel21, int leg12, int leg22,
                                 bool include_legs = true);
ActuatorPacket makeSingleWheelSpeed(std::uint8_t id, int speed);
ActuatorPacket makeSingleLegPosition(std::uint8_t id, int position);
ActuatorPacket makeServoTorque(std::uint8_t id, bool enabled);
ActuatorPacket makeWheelReturnLevel(std::uint8_t id);

// Single-owner command mailbox. All public methods and poll must run on the
// same IO/control task; BLE callbacks should queue high-level requests there.
class ActuatorCommandQueue {
public:
    bool push(const ActuatorPacket &);
    void setOutputs(int wheel11, int wheel21, int leg12, int leg22, bool legs = true);
    bool setLegTorque(bool enabled);
    void stop(bool release_legs = true);
    bool next(ActuatorPacket &out);
    bool hasPending() const { return count_ || output_pending_; }
    bool urgent() const { return urgent_; }
    std::uint32_t overflows() const { return overflows_; }
private:
    ActuatorPacket queue_[16], output_;
    std::uint8_t head_ = 0, count_ = 0;
    bool output_pending_ = false, urgent_ = false, stopped_ = false, stop_releases_ = false;
    std::uint32_t overflows_ = 0;
};

// Portable parser used unchanged by the ESP32 driver and host tests.
// Only a checksum-valid, zero-status, matching outstanding request can update
// feedback. LENGTH=8/11 carry six trailing feedback bytes, matching original
// 0x400d5b74..0x400d5c14. LENGTH=2 is a status ACK and never refreshes motion.
// Other lengths are rejected; the three extended prefix bytes are not feedback.
class ActuatorDecoder {
public:
    ActuatorDecoder();
    void expect(std::uint8_t id);
    void clearExpectation();
    std::uint8_t expectedId() const { return expected_id_; }
    bool feed(std::uint8_t byte, std::uint32_t now_ms);
    void expirePartial(std::uint32_t now_ms);
    const ActuatorSample &sample(std::uint8_t id) const;
    const ActuatorBusStats &stats() const { return stats_; }
private:
    friend class ActuatorBus;
    bool decode(std::uint32_t now_ms);
    void resetFrame();
    std::uint8_t bytes_[15] = {}, used_ = 0, expected_id_ = 0;
    std::uint32_t last_byte_ms_ = 0;
    ActuatorSample samples_[4];
    ActuatorSample unknown_;
    ActuatorBusStats stats_;
};

#ifdef ARDUINO
class ActuatorBus {
public:
    void begin();
    void poll(std::uint32_t now_ms);
    void setOutputs(int w11, int w21, int leg12, int leg22, bool legs = true) {
        commands_.setOutputs(w11,w21,leg12,leg22,legs);
    }
    void setWheelSpeeds(int w11, int w21) { setOutputs(w11,w21,0,0,false); }
    bool setWheelSpeed(std::uint8_t id, int value) { return commands_.push(makeSingleWheelSpeed(id,value)); }
    bool setLegPosition(std::uint8_t id, int value) { return commands_.push(makeSingleLegPosition(id,value)); }
    bool setServoTorque(std::uint8_t id, bool enabled) { return commands_.push(makeServoTorque(id,enabled)); }
    bool setLegTorque(bool enabled) { return commands_.setLegTorque(enabled); }
    void stop(bool release_legs = true) {
        if(retry_pending_&&!retry_safe_)retry_pending_=false;
        commands_.stop(release_legs);
    }
    bool writesPending() const { return commands_.hasPending()||setup_index_<2||retry_pending_; }
    void pauseFeedback(bool paused) { feedback_paused_=paused; }
    // Each poll transmission flushes through the physical UART. idle also waits
    // out the 1ms response guard and any outstanding read before flash work.
    bool transportIdle() const;
    const ActuatorSample &sample(std::uint8_t id) const { return decoder_.sample(id); }
    const ActuatorBusStats &stats() const { return decoder_.stats(); }
private:
    HardwareSerial *serial_ = nullptr;
    ActuatorDecoder decoder_;
    ActuatorCommandQueue commands_;
    std::uint32_t last_request_ms_ = 0;
    std::uint32_t last_tx_ms_ = 0;
    std::uint8_t schedule_index_ = 0;
    // Startup return-level configuration survives stop() queue replacement.
    std::uint8_t setup_index_ = 0;
    ActuatorPacket retry_packet_;
    bool retry_pending_=false,retry_safe_=false;
    bool started_ = false, last_was_read_ = false, feedback_paused_ = false;
};
#endif

}} // namespace rider::bringup
