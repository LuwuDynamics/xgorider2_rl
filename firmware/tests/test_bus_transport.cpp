#include "../src/actuator_bus.h"
#include "stubs/HardwareSerial.h"
#include <cstdio>
#include <cstdlib>
using namespace rider::bringup;
static unsigned checks=0;
static void check(bool v){++checks;if(!v){std::fprintf(stderr,"transport check %u FAILED\n",checks);std::exit(1);}}
HardwareSerial Serial2;
static std::uint32_t clock_ms=0,last_tx=0,pending_ms=0;
static unsigned pending_id=0,response_delay=1;
static unsigned fail_next_writes=0;
std::uint32_t millis(){return clock_ms;}
void HardwareSerial::begin(unsigned baud,unsigned config,int rx_pin,int tx_pin){
    check(baud==1000000&&config==0x0800001c&&rx_pin==13&&tx_pin==14);
}
std::size_t HardwareSerial::write(const std::uint8_t *data,std::size_t n){
    check(!pending_id); // no new TX while a response still occupies the bus
    check(sent.empty()||std::uint32_t(clock_ms-last_tx)>=1);
    if(fail_next_writes){--fail_next_writes;return 0;}
    unsigned sum=0;for(unsigned i=2;i<n;++i)sum+=data[i];
    check(data[0]==255&&data[1]==255&&(sum&255)==255&&n==unsigned(data[3])+4);
    sent.emplace_back(data,data+n);last_tx=clock_ms;
    if(data[4]==2){check(data[5]==0x24&&data[6]==6);pending_id=data[2];pending_ms=clock_ms;}
    else if(data[4]==3){
        check(data[5]==0x11||data[5]==0x18);
        const std::uint8_t ack[]={255,255,data[2],2,0,std::uint8_t(~(data[2]+2))};
        for(auto b:ack)rx.push_back(b);
    } else check(data[4]==0x83&&data[5]==0x1e&&data[6]==4);
    return n;
}
void HardwareSerial::flush(){++flushes;}
static void tick(ActuatorBus &bus){
    ++clock_ms;
    if(pending_id&&std::uint32_t(clock_ms-pending_ms)>=response_delay){
        if(response_delay<=4){
            // Mix OEM extended wheel feedback, ordinary leg feedback and the
            // short write ACKs above on the actual production poll/parser path.
            std::vector<std::uint8_t> f;
            if(pending_id==11||pending_id==21)
                f={255,255,std::uint8_t(pending_id),11,0,0xa5,0x5a,0xc3,0,2,246,255,0,0,0};
            else f={255,255,std::uint8_t(pending_id),8,0,0,2,10,0,0,0,0};
            unsigned sum=0;for(unsigned i=2;i+1<f.size();++i)sum+=f[i];f.back()=std::uint8_t(~sum);
            for(auto b:f)Serial2.rx.push_back(b);
        }
        pending_id=0;
    }
    // A missing servo response expires at the driver's 4ms timeout.
    if(pending_id&&std::uint32_t(clock_ms-pending_ms)>=4)pending_id=0;
    bus.poll(clock_ms);
}
int main(){
    ActuatorBus bus;bus.begin();
    // Main's begin();stop(true) must retain both wheel startup writes, after
    // the urgent physical stop/leg-release frames and before ordinary polling.
    bus.pauseFeedback(true);bus.stop(true);
    for(unsigned i=0;i<30&&!bus.transportIdle();++i)tick(bus);
    check(bus.transportIdle()&&Serial2.sent.size()==7);
    check(Serial2.sent[0][4]==0x83&&Serial2.sent[0].size()==18);
    for(unsigned i=1;i<5;++i)check(Serial2.sent[i][5]==0x18&&Serial2.sent[i][6]==0);
    check(Serial2.sent[5][2]==11&&Serial2.sent[5][5]==0x11&&Serial2.sent[5][6]==0);
    check(Serial2.sent[6][2]==21&&Serial2.sent[6][5]==0x11&&Serial2.sent[6][6]==0);
    bus.pauseFeedback(false);
    for(unsigned i=0;i<1000;++i){
        if(i%4==0)bus.setOutputs(100,-100,500,520);
        if(i%100==0)check(bus.setLegTorque((i/100)%2));
        response_delay=(i/50)%3+1;tick(bus);
    }
    for(auto id:{11,21,12,22})check(bus.sample(std::uint8_t(id)).sample_count>10);
    check(bus.stats().writes_sent>100&&bus.stats().requests_sent>100&&bus.stats().ack_frames>20);
    check(bus.stats().checksum_errors==0&&bus.stats().length_errors==0);
    check(bus.stats().valid_extended_frames==bus.sample(11).sample_count+bus.sample(21).sample_count);
    check(bus.sample(11).position_ticks==512&&bus.sample(21).position_ticks==512);
    check(bus.sample(11).raw_speed==-10&&bus.sample(21).raw_speed==-10);
    check(bus.sample(11).wheel_speed_mm_s<0&&bus.sample(21).wheel_speed_mm_s>0);
    check(bus.sample(12).raw_speed==10&&bus.sample(22).raw_speed==10);
    bus.pauseFeedback(true);bus.setOutputs(1000,1000,1,1);check(bus.setLegTorque(true));
    const auto begin=Serial2.sent.size();bus.stop(true);
    for(unsigned i=0;i<30&&!bus.transportIdle();++i){bus.stop(true);if(bus.writesPending())bus.setOutputs(777,777,1,1);tick(bus);}
    check(bus.transportIdle()&&!bus.writesPending());
    check(Serial2.sent.size()==begin+5);
    const auto &zero=Serial2.sent[begin];check(zero[4]==0x83&&zero.size()==18&&zero[10]==0&&zero[11]==0&&zero[15]==0&&zero[16]==0);
    for(unsigned i=1;i<5;++i){const auto &p=Serial2.sent[begin+i];check(p[4]==3&&p[5]==0x18&&p[6]==0);}
    const auto stopped=Serial2.sent.size();for(unsigned i=0;i<10;++i)tick(bus);check(Serial2.sent.size()==stopped);
    check(Serial2.flushes==Serial2.sent.size());
    // A failed motion write remains pending, but stop cancels that retry; a
    // failed stop write must instead be retried before subsequent commands.
    bus.setWheelSpeeds(321,321);fail_next_writes=1;tick(bus);
    check(bus.writesPending()&&bus.stats().tx_errors==1);
    const auto failure_start=Serial2.sent.size();bus.stop(true);fail_next_writes=1;tick(bus);
    check(bus.writesPending()&&bus.stats().tx_errors==2);
    for(unsigned i=0;i<30&&!bus.transportIdle();++i){bus.stop(true);tick(bus);}
    check(bus.transportIdle()&&Serial2.sent.size()==failure_start+5);
    check(Serial2.sent[failure_start][10]==0&&Serial2.sent[failure_start][15]==0);
    bus.pauseFeedback(false);response_delay=10;
    for(unsigned i=0;i<30;++i)tick(bus);
    check(bus.stats().response_timeouts>0);
    bus.pauseFeedback(true);bus.stop(true);
    for(unsigned i=0;i<30&&!bus.transportIdle();++i)tick(bus);
    check(bus.transportIdle());
    std::printf("{\"test\":\"bus_transport_mock\",\"checks\":%u,\"result\":\"PASS\"}\n",checks);
}
