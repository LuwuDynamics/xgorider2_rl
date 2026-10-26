#include "board_support.h"
#include "battery_tables.h"
#include <cmath>
#ifdef ARDUINO
#include <Arduino.h>
#include <driver/rmt.h>
#endif

namespace rider { namespace board {
float batteryVoltage(std::uint16_t adc) {
    return float((((double(adc)*0.000244140625)*3.3)*3.0)*double(1.034999966621399f));
}
float batteryPercent(float voltage) {
    unsigned index=0;
    while(index<126 && voltage>float(kCellVoltages[index]+kCellVoltages[index]))++index;
    const float result=kBatteryFractions[index]*100.0f;
    return result<0?0:result>100?100:result;
}
void BatteryMonitor::update(std::uint16_t adc,std::uint32_t uptime_ms) {
    // 0x400d29e8 precedes the ADC conversion, so it observes previous percent.
    if(state_.percent<=7 && state_.low_stage==0) {
        const auto previous=low_count_++;
        if(previous>=200){state_.low_stage=1;low_count_=0;}
    }
    if(state_.percent>7 && low_count_)low_count_=0;
    if(state_.low_stage) {
        if(state_.low_stage>30)state_.low_stage=30;
        const auto previous=blink_count_++;
        if(previous>79){++state_.low_stage;blink_count_=0;state_.low_blink=!state_.low_blink;}
    }
    state_.neutralize=state_.low_stage>0;
    state_.stop_required=state_.low_stage>=10;
    state_.release_legs=state_.low_stage>=12;
    state_.adc_raw=adc;
    readings_[index_++]=batteryVoltage(adc);
    if(index_!=15)return;
    index_=0;
    float mean=0;
    for(float reading:readings_) {
        const float value=(uptime_ms<5000 || double(std::fabs(reading-state_.voltage))<0.1)
                          ?reading:state_.voltage;
        mean=float(mean+float(value/15.0f));
    }
    state_.voltage=mean;state_.percent=batteryPercent(mean);
    state_.battery_valid=true;++state_.battery_updates;
}
CalibrationSwitchEvent CalibrationSwitch::update(bool high) {
    // Original does not clear mismatches on matching readings.
    if(high!=stable_high_)++mismatches_;
    if(mismatches_<=50)return CalibrationSwitchEvent::None;
    mismatches_=0;stable_high_=high;
    return high?CalibrationSwitchEvent::Save:CalibrationSwitchEvent::Enter;
}
Rgb paletteColor(std::uint8_t index) {
    unsigned p=255-index;
    if(p<85)return Rgb{std::uint8_t(255-p*3),0,std::uint8_t(p*3)};
    if(p<170){p-=85;return Rgb{0,std::uint8_t(p*3),std::uint8_t(255-p*3)};}
    p-=170;return Rgb{std::uint8_t(p*3),std::uint8_t(255-p*3),0};
}
std::uint8_t brightnessScale(std::uint8_t channel) { return std::uint8_t((unsigned(channel)*129)>>8); }
bool renderLeds(const LedControl &c,std::uint32_t now,const LedPixels &user,LedPixels &out) {
    Rgb uniform{};
    bool fill=true;
    if(c.robot_mode==250)uniform.r=c.low_blink?200:0;
    else if(c.robot_mode==0)uniform.r=200;
    else if(c.pitch_trim_stage==2)uniform.b=now%1000<500?128:0;
    else if(c.roll_period!=0) {
        const int value=std::isfinite(c.roll_request)?int(double(c.roll_request)*0.0625*127.0+128.0):128;
        const auto shade=std::uint8_t(value);
        for(unsigned i=0;i<4;++i)out[i]=i<2?Rgb{0,std::uint8_t(shade^255),shade}:Rgb{0,shade,std::uint8_t(shade^255)};
        fill=false;
    } else if(c.height_period!=0) {
        const float scaled=(c.height_request/36.0f)*255.0f;
        const auto shade=std::uint8_t(std::isfinite(scaled)?unsigned(scaled<0?0:scaled):0);
        uniform=Rgb{0,shade,std::uint8_t(shade^255)};
    } else if(c.robot_mode==1) {
        if(std::fabs(c.forward_request)>5 || std::fabs(c.turn_request)>5) {
            out={};
            if(c.forward_request>5){out[0].g=240;out[3].g=240;}
            if(c.forward_request<-5){out[1].g=240;out[2].g=240;}
            if(c.turn_request>5){out[0].g=240;out[1].g=240;}
            if(c.turn_request<-5){out[2].g=240;out[3].g=240;}
        } else out=user;
        fill=false;
    } else return false;
    if(fill)out.fill(uniform);
    return true;
}

bool renderFaultLeds(const LedControl &c,std::uint32_t now,LedPixels &out) {
    if(c.fault==LedFault::None||c.robot_mode==1||c.robot_mode==250)return false;
    Rgb color{};
    switch(c.fault) {
        case LedFault::Tilt:color.r=200;break;
        case LedFault::Fall:color.r=now%200<100?255:0;break;
        case LedFault::Other:color.r=now%1000<500?200:0;break;
        case LedFault::Manual:color=Rgb{200,70,0};break;
        default:return false;
    }
    out.fill(color);
    return true;
}

#ifdef ARDUINO
bool BoardSupport::begin() {
    // 0x400d292c: INPUT_PULLUP=5, ANALOG=9, OUTPUT=3.
    pinMode(kCalibrationPin,INPUT_PULLUP);pinMode(kBatteryPin,ANALOG);pinMode(kLedPin,OUTPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(kBatteryPin,ADC_11db);
    rmt_config_t config{};
    config.rmt_mode=RMT_MODE_TX;config.channel=RMT_CHANNEL_0;
    config.gpio_num=GPIO_NUM_27;config.clk_div=4;config.mem_block_num=1;
    config.tx_config.loop_en=false;config.tx_config.carrier_en=false;
    config.tx_config.idle_output_en=true;config.tx_config.idle_level=RMT_IDLE_LEVEL_LOW;
    leds_ready_=rmt_config(&config)==ESP_OK && rmt_driver_install(RMT_CHANNEL_0,0,0)==ESP_OK;
    last_poll_ms_=millis();last_led_ms_=last_poll_ms_;started_=true;
    fillPalette(0,4,234);show(user_);
    return leds_ready_;
}
void BoardSupport::poll(std::uint32_t now) {
    if(!started_||std::uint32_t(now-last_poll_ms_)<2)return;
    last_poll_ms_=now;
    battery_.update(std::uint16_t(analogRead(kBatteryPin)),now);
    const auto e=switch_.update(digitalRead(kCalibrationPin)!=0);
    if(e!=CalibrationSwitchEvent::None)event_=e;
}
BoardState BoardSupport::state() const {
    auto result=battery_.state();result.calibration_pressed=switch_.pressed();return result;
}
CalibrationSwitchEvent BoardSupport::takeCalibrationEvent() {
    const auto result=event_;event_=CalibrationSwitchEvent::None;return result;
}
bool BoardSupport::setUserLed(std::uint8_t index,Rgb color) {
    if(index>=user_.size())return false;
    user_[index]=color;return true;
}
void BoardSupport::fillPalette(std::uint8_t first,std::uint8_t count,std::uint8_t color) {
    const unsigned end=count?unsigned(first)+count:4;
    for(unsigned i=first;i<4&&i<end;++i)user_[i]=paletteColor(color);
}
void BoardSupport::updateLeds(const LedControl &control,std::uint32_t now) {
    if(std::uint32_t(now-last_led_ms_)<20)return;
    last_led_ms_=now;LedPixels pixels;
    if(renderFaultLeds(control,now,pixels)||renderLeds(control,now,user_,pixels))show(pixels);
}
void BoardSupport::show(const LedPixels &pixels) {
    if(!leds_ready_)return;
    rmt_item32_t items[97]{};unsigned bit=0;
    for(const auto &pixel:pixels) {
        const std::uint8_t channels[]={brightnessScale(pixel.g),brightnessScale(pixel.r),brightnessScale(pixel.b)};
        for(auto channel:channels)for(int shift=7;shift>=0;--shift) {
            const bool one=channel&(1u<<shift);
            items[bit].level0=1;items[bit].duration0=one?16:8;
            items[bit].level1=0;items[bit++].duration1=one?9:17;
        }
    }
    items[96].level0=0;items[96].duration0=3000;items[96].level1=0;items[96].duration1=3000;
    if(rmt_write_items(RMT_CHANNEL_0,items,97,true)==ESP_OK)shown_=pixels;
}
#endif
}} // namespace rider::board
