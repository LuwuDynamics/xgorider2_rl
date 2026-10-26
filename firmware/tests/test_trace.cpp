#include "trace.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
using namespace rider2::trace;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)

static unsigned commas(const std::string& text){unsigned n=0;for(char c:text)n+=c==',';return n;}

int main(){
    // The BLE sample layout is what tools/capture_trace.py unpacks as '<14h': 28 bytes, the fixed field order.
    CHECK(sizeof(Sample)==28);
    Sample s{};s.pitch=725;s.pitch_rate=-980;s.roll=181;s.yaw_rate=-2053;s.cmd11=104;s.cmd21=-180;
    s.sp11=5;s.sp21=-1;s.fsp11=8;s.fsp21=0;s.leg_a=592;s.leg_b=420;s.a0=103;s.a1=-180;
    Extra x{};x.ms=123456;x.load11=-12;x.load21=1030;x.pos11=1023;x.pos21=0;x.acc_x=-981;x.acc_y=5;x.acc_z=32767;x.flags=5;
    x.age11=3;x.age21=-1;x.age12=32767;x.age22=26;x.dt_us=6012;
    x.load12=140;x.load22=-7;x.pos12=562;x.pos22=449;x.vbat_adc=3150;

    char line[256];
    const std::size_t n=formatLine(line,sizeof line,70000u,s,x);
    CHECK(n>0&&n==std::strlen(line));
    CHECK(std::string(line)=="T,70000,123456,725,-980,181,-2053,104,-180,5,-1,8,0,592,420,103,-180,-12,1030,1023,0,-981,5,32767,5,3,-1,32767,26,6012,140,-7,562,449,3150\n");
    // 34 fields, exactly as many as the header names after "# trace ".
    const std::string header(kCsvHeader);
    CHECK(header.rfind("# trace ",0)==0);
    CHECK(commas(header)==33);
    CHECK(commas(line)==commas(header)+1);   // "T" is the extra leading column
    CHECK(commas(line)==34);

    // Extremes still fit in the 256-byte buffer the firmware uses.
    Sample big{};Extra bigx{};
    big.pitch=big.pitch_rate=big.roll=big.yaw_rate=big.cmd11=big.cmd21=std::numeric_limits<std::int16_t>::min();
    big.sp11=big.sp21=big.fsp11=big.fsp21=big.leg_a=big.leg_b=big.a0=big.a1=std::numeric_limits<std::int16_t>::min();
    bigx.ms=std::numeric_limits<std::uint32_t>::max();
    bigx.load11=bigx.load21=bigx.pos11=bigx.pos21=bigx.acc_x=bigx.acc_y=bigx.acc_z=bigx.flags=std::numeric_limits<std::int16_t>::min();
    bigx.age11=bigx.age21=bigx.age12=bigx.age22=std::numeric_limits<std::int16_t>::min();bigx.dt_us=std::numeric_limits<std::uint16_t>::max();
    bigx.load12=bigx.load22=bigx.pos12=bigx.pos22=std::numeric_limits<std::int16_t>::min();bigx.vbat_adc=std::numeric_limits<std::uint16_t>::max();
    CHECK(formatLine(line,sizeof line,std::numeric_limits<std::uint32_t>::max(),big,bigx)>0);
    // A buffer that is too small is reported, never overrun.
    char tiny[16];
    CHECK(formatLine(tiny,sizeof tiny,1,s,x)==0);
    CHECK(formatLine(line,0,1,s,x)==0);

    // One-shot dump of a ring that has not wrapped yet: oldest sample is index 0, ends at the head seen at start.
    UsbCursor c;uint32_t index=0;
    CHECK(!c.take(10,1200,index));            // off: nothing
    c.dump(5,1200);
    for(uint32_t want=0;want<5;++want){CHECK(c.take(5,1200,index)&&index==want);}
    CHECK(!c.take(5,1200,index));
    CHECK(c.finished&&c.mode==0&&c.sent==5);
    // Samples written while the dump runs do not extend it.
    c=UsbCursor();c.dump(3,1200);
    CHECK(c.take(7,1200,index)&&index==0);
    CHECK(c.take(9,1200,index)&&index==1);
    CHECK(c.take(9,1200,index)&&index==2);
    CHECK(!c.take(9,1200,index)&&c.finished&&c.sent==3);

    // A full ring starts at the oldest slot still held.
    c=UsbCursor();c.dump(5000,1200);
    CHECK(c.take(5000,1200,index)&&index==3800);
    // A dump reader that has been lapped jumps to the oldest sample that is still intact ...
    c=UsbCursor();c.dump(1300,1200);          // next=100, end=1300
    CHECK(c.take(1500,1200,index)&&index==300);
    // ... and if everything the dump wanted has been overwritten, the dump just ends.
    c=UsbCursor();c.dump(1300,1200);
    CHECK(!c.take(2600,1200,index));
    CHECK(c.finished&&c.mode==0);

    // Continuous stream: starts at "now", follows the head, never goes backwards.
    c=UsbCursor();c.stream(40);
    CHECK(!c.take(40,1200,index));
    CHECK(!c.finished&&c.mode==2);
    CHECK(c.take(42,1200,index)&&index==40);
    CHECK(c.take(42,1200,index)&&index==41);
    CHECK(!c.take(42,1200,index));
    CHECK(c.take(43,1200,index)&&index==42);
    CHECK(c.sent==3);
    c.stop();
    CHECK(!c.take(100,1200,index));
    // Falling behind in stream mode drops the old samples rather than reading reused slots.
    c=UsbCursor();c.stream(0);
    CHECK(c.take(2000,1200,index)&&index==800);
    // The 32-bit counters may wrap.
    c=UsbCursor();c.stream(0xfffffffeu);
    CHECK(c.take(0xffffffffu,1200,index)&&index==0xfffffffeu);
    CHECK(c.take(2,1200,index)&&index==0xffffffffu);
    CHECK(c.take(2,1200,index)&&index==0u);
    CHECK(c.take(2,1200,index)&&index==1u);
    CHECK(!c.take(2,1200,index));

    std::printf("trace: %u checks passed\n",checks);
}
