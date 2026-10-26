#include "../src/actuator_bus.h"
#include "../src/board_support.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
using namespace rider::bringup;
using namespace rider::board;
static unsigned checks=0;
static void check(bool v){++checks;if(!v){std::fprintf(stderr,"FAILED %u\n",checks);std::exit(1);}}
static bool valid(const ActuatorPacket &p){unsigned s=0;for(unsigned i=2;i<p.size;++i)s+=p.bytes[i];return p.size>=8&&(s&255)==255;}
static int word(const ActuatorPacket&p,unsigned o){unsigned w=p.bytes[o]+unsigned(p.bytes[o+1])*256;return w<32768?int(w):int(w)-65536;}
int main(){
    auto p=makeActuatorOutput(-2000,2000,-1,9999);
    check(valid(p)&&p.size==28&&p.bytes[4]==0x83&&p.bytes[5]==0x1e&&p.bytes[6]==4);
    check(p.bytes[7]==11&&word(p,10)==-1023&&p.bytes[12]==21&&word(p,15)==1023);
    check(p.bytes[17]==12&&word(p,18)==0&&p.bytes[22]==22&&word(p,23)==1023);
    check(word(p,8)==0&&word(p,13)==0&&word(p,20)==0&&word(p,25)==0);
    check(makeSingleWheelSpeed(12,5).size==0&&makeSingleLegPosition(11,500).size==0);
    for(unsigned id=0;id<256;++id){
        const bool allowed=id==11||id==21||id==12||id==22;
        const auto torque=makeServoTorque(std::uint8_t(id),true);
        check((torque.size!=0)==allowed);
        if(allowed)check(valid(torque)&&torque.bytes[4]==3&&torque.bytes[5]==0x18&&torque.bytes[6]==1);
    }
    ActuatorCommandQueue queue;
    queue.setOutputs(1,2,300,400);queue.setOutputs(3,4,500,600);
    check(queue.next(p)&&word(p,10)==3&&word(p,15)==4&&word(p,18)==500);
    check(!queue.next(p));
    queue.setOutputs(700,800,300,400);check(queue.setLegTorque(true));queue.stop(true);
    // Repeated stop cannot cancel torque-off commands or revive queued motion.
    for(unsigned i=0;i<5;++i){
        queue.stop(true);queue.setOutputs(999,999,200,200);check(queue.next(p)&&valid(p));
        if(i==0)check(p.size==18&&word(p,10)==0&&word(p,15)==0);
        else check(p.bytes[5]==0x18&&p.bytes[6]==0&&p.bytes[2]==(i%2?22:12));
    }
    check(!queue.next(p));
    queue.setOutputs(10,20,30,40);check(queue.next(p)&&word(p,10)==10);
    for(unsigned i=0;i<16;++i)check(queue.push(makeServoTorque(12,true)));
    check(!queue.push(makeServoTorque(12,true))&&queue.overflows()==1);
    check(!queue.setLegTorque(true));queue.stop(false);
    check(queue.next(p)&&p.size==18&&!queue.next(p));
    queue.stop(true);
    for(unsigned i=0;i<5;++i)check(queue.next(p)&&valid(p));
    check(!queue.next(p));
    CalibrationSwitch sw;
    for(unsigned i=0;i<50;++i)check(sw.update(true)==CalibrationSwitchEvent::None);
    check(sw.update(true)==CalibrationSwitchEvent::Save&&!sw.pressed());
    for(unsigned i=0;i<50;++i){check(sw.update(false)==CalibrationSwitchEvent::None);check(sw.update(true)==CalibrationSwitchEvent::None);}
    check(sw.update(false)==CalibrationSwitchEvent::Enter&&sw.pressed());
    BatteryMonitor battery;
    for(unsigned i=0;i<15;++i)battery.update(0,i*2);
    check(battery.state().battery_valid&&battery.state().percent==2.0f);
    for(unsigned i=0;i<200;++i)battery.update(0,30+i*2);
    check(battery.state().low_stage==0);
    battery.update(0,430);check(battery.state().low_stage==1&&battery.state().neutralize);
    for(unsigned i=0;i<729;++i)battery.update(0,432+i*2);
    check(battery.state().stop_required&&battery.state().low_stage>=10);
    for(unsigned i=0;i<162;++i)battery.update(0,2000+i*2);
    check(battery.state().release_legs);
    for(unsigned v=0;v<4096;++v)check(std::isfinite(batteryVoltage(v))&&batteryPercent(batteryVoltage(v))>=2&&batteryPercent(batteryVoltage(v))<=100);
    check(brightnessScale(255)==128&&brightnessScale(0)==0&&brightnessScale(200)==100);
    check(paletteColor(0).r==255&&paletteColor(85).g==255&&paletteColor(170).b==255);
    std::printf("{\"test\":\"board_bus_full\",\"checks\":%u,\"result\":\"PASS\"}\n",checks);
}
