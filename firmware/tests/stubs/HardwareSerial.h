#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
class HardwareSerial {
public:
    void setRxBufferSize(std::size_t){}
    void begin(unsigned,unsigned,int,int);
    int available(){return int(rx.size());}
    int read(){if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
    std::size_t write(const std::uint8_t*,std::size_t);
    void flush();
    std::deque<std::uint8_t> rx;
    std::vector<std::vector<std::uint8_t>> sent;
    unsigned flushes=0;
};
extern HardwareSerial Serial2;
