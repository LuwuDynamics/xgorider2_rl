#include "command_protocol.h"
#ifdef NDEBUG
#error Protocol tests require assertions; compile with -UNDEBUG.
#endif
#include <cassert>
#include <cstring>
#include <cstdio>
#include <random>
using namespace rider2::protocol;
static bool send(StreamParser &p,Source source,const uint8_t *bytes,size_t n,uint32_t time,CommandEvent &out) {
    bool ok=false;for(size_t i=0;i<n;++i)if(p.feed(source,bytes[i],time,out))ok=true;return ok;
}
int main() {
    StreamParser p;CommandEvent e;uint8_t frame[64];uint8_t value=201;
    size_t n=buildFrame(1,0x30,&value,1,frame,sizeof frame);assert(n==9);
    assert(send(p,Source::Usb,frame,n,100,e)&&e.reg==0x30&&e.data[0]==201);
    assert(commandKind(e)==CommandKind::Forward);
    // Concurrent fragmented transports must never share a partial packet.
    assert(!send(p,Source::Usb,frame,4,200,e));assert(p.receiving(Source::Usb));
    assert(send(p,Source::Aux,frame,n,200,e));assert(e.source==Source::Aux);
    assert(send(p,Source::Usb,frame+4,n-4,201,e));assert(e.source==Source::Usb);
    frame[n-3]^=1;assert(!send(p,Source::Usb,frame,n,300,e));frame[n-3]^=1;
    frame[n-1]=0xab;assert(!send(p,Source::Usb,frame,n,300,e));frame[n-1]=0xaa;
    assert(send(p,Source::Usb,frame,n,300,e));
    assert(!send(p,Source::Usb,frame,4,400,e));
    assert(!send(p,Source::Usb,frame+4,n-4,501,e));
    assert(send(p,Source::Usb,frame,n,502,e));
    assert(!send(p,Source::Usb,frame,1,550,e));assert(p.receiving(Source::Usb));
    p.expire(Source::Usb,651);assert(!p.receiving(Source::Usb));
    // Both original write types have exactly the same register semantics.
    for(uint8_t type=0;type<2;++type)for(uint8_t reg:{3,4,5,6,8,9,10,0x30,0x32,0x35,0x36,0x39,0x3e,0x60,0x61,0x82,0xa0,0xa1}) {
        n=buildFrame(type,reg,&value,1,frame,sizeof frame);assert(send(p,Source::Ble,frame,n,600,e));
    }
    uint8_t payload[24]={};
    for(uint8_t reg=0x69;reg<=0x6c;++reg)for(size_t size:{3u,12u}) {
        n=buildFrame(1,reg,payload,size,frame,sizeof frame);assert(send(p,Source::Ble,frame,n,700,e));
    }
    for(size_t size:{0u,2u,4u,11u,13u}) {
        n=buildFrame(1,0x69,payload,size,frame,sizeof frame);assert(!send(p,Source::Ble,frame,n,700,e));
    }
    n=buildFrame(1,0x13,payload,20,frame,sizeof frame);assert(send(p,Source::Ble,frame,n,700,e));
    n=buildFrame(1,0x13,payload,21,frame,sizeof frame);assert(!send(p,Source::Ble,frame,n,700,e));
    Telemetry t;t.battery_percent=87;t.mode=1;t.roll_deg=-12.75f;t.pitch_deg=45.25f;t.yaw_deg=123;
    for(uint8_t reg:{1,2,7,11,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0xb0,0xb1,0xb2}) {
        const uint8_t requestCount=255;
        n=buildFrame(2,reg,&requestCount,1,frame,sizeof frame);assert(send(p,Source::Usb,frame,n,800,e));
        uint8_t reply[64];const size_t count=buildReadResponse(e,t,reply,sizeof reply);
        assert(count>=9&&count<=32&&reply[3]==0x12&&reply[4]==reg&&reply[count-1]==0xaa);
        uint8_t sum=0;for(size_t i=2;i<count-2;++i)sum=uint8_t(sum+reply[i]);assert(sum==255);
    }
    n=buildStreamTelemetry(t,frame,sizeof frame);assert(n==45&&frame[0]==0x55&&frame[2]==87&&frame[44]==0xaa);
    assert(buildStreamTelemetry(t,frame,44)==0);
    t.controller_kind=1;t.wheel_source=2;t.policy_mode=2;t.last_stop_reason=11;
    t.policy_calls=0x12345678;t.policy_failures=0x11223344;t.rl_output_ticks=0xabcdef01;
    e.type=2;e.reg=0xb2;e.length=1;e.data[0]=255;
    n=buildReadResponse(e,t,frame,sizeof frame);assert(n==24);
    const uint8_t expected[]={1,2,2,11,0x78,0x56,0x34,0x12,0x44,0x33,0x22,0x11,1,0xef,0xcd,0xab};
    assert(!std::memcmp(frame+5,expected,sizeof expected));
    assert(buildReadResponse(e,t,frame,23)==0);
    // Byte fuzzing plus injected valid frames verifies recovery without OOB or
    // bogus event emission; event payload length must always pass validation.
    std::mt19937 rng(116);
    for(unsigned i=0;i<100000;++i) {
        if(p.feed(Source::Usb,uint8_t(rng()),i,e))assert(validCommand(e));
        if(i%1000==0){p.reset(Source::Usb);n=buildFrame(1,0x30,&value,1,frame,sizeof frame);assert(send(p,Source::Usb,frame,n,i,e));}
    }
    std::puts("protocol framing, fragmentation, source isolation, invalid sizes, checksums and 100000 fuzz bytes passed");
}
