#include "device_settings.h"
#include "esp_partition.h"
#include "esp_spi_flash.h"
#ifdef NDEBUG
#error Settings tests require assertions enabled.
#endif
#include <cassert>
#include <cstring>
#include <cstdio>
#include <limits>
#include <vector>
static std::vector<uint8_t> flash(0x400000,0xa5);
static esp_partition_t partitions[]={{0x9000,0x5000},{0x190000,0x180000},{0x330000,0xd0000}};
static bool readFail=false,writeFail=false,noPartitions=false;
static unsigned eraseCount=0,writeCount=0;
esp_partition_iterator_t esp_partition_find(int,int,const char *){return noPartitions?nullptr:partitions;}
const esp_partition_t *esp_partition_get(esp_partition_iterator_t it){return it;}
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t it){return it==partitions+2?nullptr:it+1;}
void esp_partition_iterator_release(esp_partition_iterator_t){}
int spi_flash_read(uint32_t address,void *out,size_t length){
    assert(address%4==0&&length%4==0&&address+length<=flash.size());
    if(readFail)return -1;std::memcpy(out,flash.data()+address,length);return 0;
}
int spi_flash_write(uint32_t address,const void *data,size_t length){
    assert((address==0x310000||address==0x320000||address==0x321000)&&length==4096);
    ++writeCount;if(writeFail)return -1;std::memcpy(flash.data()+address,data,length);return 0;
}
int spi_flash_erase_sector(size_t sector){
    assert(sector==0x310||sector==0x320||sector==0x321);++eraseCount;
    std::memset(flash.data()+sector*4096,0xff,4096);return 0;
}
int main(){
    using namespace rider2;
    assert(saveLegCalibration(500,530));uint32_t words[3];std::memcpy(words,flash.data()+0x310000,12);
    assert(words[0]==500&&words[1]==530&&words[2]==1234);
    for(size_t i=12;i<4096;++i)assert(flash[0x310000+i]==0xa5);
    unsigned erases=eraseCount;assert(saveLegCalibration(500,530)&&eraseCount==erases);
    assert(!saveLegCalibration(1024,500)&&eraseCount==erases);
    partitions[1].size+=4;assert(!saveLegCalibration(501,530)&&eraseCount==erases);partitions[1].size-=4;
    noPartitions=true;assert(!saveLegCalibration(501,530)&&eraseCount==erases);noPartitions=false;
    readFail=true;assert(!saveLegCalibration(501,530)&&eraseCount==erases);readFail=false;
    const uint8_t name[]={'1','2','3','4'};
    assert(saveBleSuffix(name,4));auto n=readBleName();assert(n.valid&&n.length==4&&!std::strcmp(n.full_name,"XGORider_1234"));
    for(size_t i=24;i<4096;++i)assert(flash[0x320000+i]==0xa5);
    assert(!saveBleSuffix(name,0)&&!saveBleSuffix(name,21));
    const uint8_t bad[]={0,'x'};assert(!saveBleSuffix(bad,2));
    flash[0x320000]=255;n=readBleName();assert(!n.valid&&!std::strcmp(n.full_name,"XGORider_RECOVERED"));
    const double trim=1.234;uint64_t bits;std::memcpy(&bits,&trim,8);
    assert(savePitchTrim(trim));uint32_t low;std::memcpy(&low,flash.data()+0x321000,4);assert(low==uint32_t(bits));
    for(size_t i=4;i<16;++i)assert(flash[0x321000+i]==0xa5);
    for(size_t i=36;i<4096;++i)assert(flash[0x321000+i]==0xa5);
    auto t=loadPitchTrim();assert(t.valid&&t.extended&&!t.extension_invalid&&t.value==trim);
    flash[0x321020]^=1;t=loadPitchTrim();assert(t.valid&&!t.extended&&t.extension_invalid&&t.value!=trim);
    // CRC damage, unsupported version, missing legacy data all fail closed or
    // explicitly fall back to the legacy representation.
    assert(savePitchTrim(-7.5));t=loadPitchTrim();assert(t.valid&&t.extended&&t.value==-7.5);
    flash[0x321014]=2;t=loadPitchTrim();assert(t.valid&&!t.extended&&t.extension_invalid);
    std::memset(flash.data()+0x321000,0xff,36);t=loadPitchTrim();assert(t.valid&&t.defaulted&&!t.extended&&t.value==0);
    assert(savePitchTrim(trim));flash[0x321020]^=1;std::memset(flash.data()+0x321000,0xff,4);
    t=loadPitchTrim();assert(t.valid&&t.defaulted&&!t.extended&&t.extension_invalid&&t.value==0);
    std::memset(flash.data()+0x321010,0xff,20);
    const uint32_t legacy=0x12345678;std::memcpy(flash.data()+0x321000,&legacy,4);
    t=loadPitchTrim();assert(t.valid&&!t.extended&&t.legacy_low_word==legacy);
    readFail=true;t=loadPitchTrim();assert(!t.valid&&t.read_error!=0);readFail=false;
    assert(!savePitchTrim(8)&&!savePitchTrim(std::numeric_limits<double>::infinity()));
    writeFail=true;assert(!saveLegCalibration(501,531));writeFail=false;
    assert(flash[0x30ffff]==0xa5&&flash[0x311000]==0xa5&&flash[0x31ffff]==0xa5&&flash[0x322000]==0xa5);
    assert(writeCount==eraseCount);
    std::puts("settings format, name bounds, trim low32/extended64/CRC/fallback, partition overlap, unchanged skip, sector preservation and flash failures passed");
}
