#include "device_settings.h"
#include <esp_spi_flash.h>
#include <esp_partition.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace rider2 {
namespace {
constexpr uint32_t kLegSector=0x310000,kNameSector=0x320000,kTrimSector=0x321000;
constexpr size_t kSectorSize=4096;
constexpr char kPrefix[]="XGORider_"; // 0x400d4b50 -> 0x3f400204
constexpr uint32_t kTrimMagic=0x32525450; // little-endian "PTR2"
constexpr uint32_t kTrimVersion=1;
uint32_t crc32(const uint8_t *bytes,size_t length) {
    uint32_t crc=0xffffffff;
    for(size_t i=0;i<length;++i) {
        crc^=bytes[i];for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320u&uint32_t(-int32_t(crc&1)));
    }
    return ~crc;
}
uint32_t wordAt(const uint8_t *bytes) {uint32_t word;std::memcpy(&word,bytes,4);return word;}
bool validSuffix(const uint8_t *suffix,size_t n) {
    if(!suffix || n<1 || n>20)return false;
    for(size_t i=0;i<n;++i)if(suffix[i]<0x20 || suffix[i]==0x7f)return false;
    return true;
}
bool isSettingsGap(uint32_t address) {
    if(address!=kLegSector && address!=kNameSector && address!=kTrimSector)return false;
    esp_partition_iterator_t it=esp_partition_find(ESP_PARTITION_TYPE_ANY,ESP_PARTITION_SUBTYPE_ANY,nullptr);
    bool saw_partition=false;
    while(it) {
        const esp_partition_t *p=esp_partition_get(it);saw_partition=true;
        if(address<uint64_t(p->address)+p->size && uint64_t(address)+kSectorSize>p->address) {
            esp_partition_iterator_release(it);return false;
        }
        it=esp_partition_next(it);
    }
    return saw_partition;
}
bool writeSettings(uint32_t address,const void *data,size_t length) {
    if(!data || length>kSectorSize || length%4 || !isSettingsGap(address))return false;
    uint8_t *sector=static_cast<uint8_t *>(std::malloc(kSectorSize));
    if(!sector)return false;
    bool ok=spi_flash_read(address,sector,kSectorSize)==ESP_OK;
    if(ok && std::memcmp(sector,data,length)==0) {std::free(sector);return true;}
    if(ok) {
        std::memcpy(sector,data,length);
        ok=spi_flash_erase_sector(address/kSectorSize)==ESP_OK &&
           spi_flash_write(address,sector,kSectorSize)==ESP_OK;
    }
    // Read back the whole sector; bytes outside the setting were preserved.
    alignas(4) uint8_t verify[128];
    for(size_t off=0;ok && off<kSectorSize;off+=sizeof verify)
        ok=spi_flash_read(address+off,verify,sizeof verify)==ESP_OK &&
           std::memcmp(verify,sector+off,sizeof verify)==0;
    std::free(sector);return ok;
}
}
BleName readBleName() {
    BleName n;alignas(4) uint8_t raw[24]={};
    n.read_error=spi_flash_read(kNameSector,raw,sizeof raw);
    n.valid=n.read_error==ESP_OK && validSuffix(raw+1,raw[0]);
    if(n.valid) {n.length=raw[0];std::memcpy(n.suffix,raw+1,n.length);}
    else {std::memcpy(n.suffix,"RECOVERED",10);n.length=9;}
    std::memcpy(n.full_name,kPrefix,sizeof kPrefix-1);
    std::memcpy(n.full_name+sizeof kPrefix-1,n.suffix,n.length);
    return n;
}
bool saveLegCalibration(uint16_t a,uint16_t b) {
    if(a>1023 || b>1023)return false;
    const uint32_t words[]={a,b,1234};
    return writeSettings(kLegSector,words,sizeof words);
}
bool savePitchTrim(double trim) {
    static_assert(sizeof(double)==8,"Original trim format requires IEEE binary64");
    if(!std::isfinite(trim) || std::fabs(trim)>7.5)return false;
    uint64_t bits;std::memcpy(&bits,&trim,sizeof bits);
    const uint32_t low=uint32_t(bits);
    alignas(4) uint8_t prefix[36];
    if(spi_flash_read(kTrimSector,prefix,sizeof prefix)!=ESP_OK)return false;
    // Keep original bytes0..3 for old firmware; bytes4..15 are untouched.
    std::memcpy(prefix,&low,4);
    std::memcpy(prefix+16,&kTrimMagic,4);std::memcpy(prefix+20,&kTrimVersion,4);
    std::memcpy(prefix+24,&bits,8);
    const uint32_t crc=crc32(prefix+16,16);std::memcpy(prefix+32,&crc,4);
    return writeSettings(kTrimSector,prefix,sizeof prefix);
}
TrimSetting loadPitchTrim() {
    TrimSetting t;alignas(4) uint8_t prefix[36]={};
    t.read_error=spi_flash_read(kTrimSector,prefix,sizeof prefix);
    if(t.read_error!=ESP_OK)return t;
    t.legacy_low_word=wordAt(prefix);
    if(wordAt(prefix+16)==kTrimMagic) {
        double value;std::memcpy(&value,prefix+24,8);
        const bool valid=wordAt(prefix+20)==kTrimVersion &&
                         wordAt(prefix+32)==crc32(prefix+16,16) &&
                         std::isfinite(value) && std::fabs(value)<=7.5;
        if(valid) {t.valid=true;t.extended=true;t.value=value;return t;}
        t.extension_invalid=true;
    }
    if(t.legacy_low_word==UINT32_MAX) {
        // Stock machines can have valid leg zeros but have never calibrated
        // pitch trim. Missing optional trim is zero, never an arming dead end.
        t.valid=true;t.defaulted=true;t.value=0;return t;
    }
    const uint64_t bits=t.legacy_low_word;std::memcpy(&t.value,&bits,8);
    t.valid=std::isfinite(t.value) && std::fabs(t.value)<=7.5;
    if(!t.valid)t.value=0;
    return t;
}
bool saveBleSuffix(const uint8_t *suffix,size_t length) {
    if(!validSuffix(suffix,length))return false;
    alignas(4) uint8_t bytes[24]={};bytes[0]=uint8_t(length);
    std::memcpy(bytes+1,suffix,length);
    return writeSettings(kNameSector,bytes,sizeof bytes);
}
}
