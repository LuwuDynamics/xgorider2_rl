#include "ble_transport.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_bt_main.h>
#include <atomic>
#include <algorithm>

namespace rider2 {
namespace {
constexpr const char *kService="0000fff0-0000-1000-8000-00805f9b34fb";
constexpr const char *kNotify="0000fff1-0000-1000-8000-00805f9b34fb";
constexpr const char *kWrite="0000fff2-0000-1000-8000-00805f9b34fb";
QueueHandle_t rxQueue=nullptr;
BLEServer *server=nullptr;
BLECharacteristic *tx=nullptr;
bool ready=false;
std::atomic<bool> connected{false},restartAdvertising{false};
std::atomic<uint32_t> dropped{0};
class ServerCallbacks final:public BLEServerCallbacks {
    void onConnect(BLEServer *) override {if(rxQueue)xQueueReset(rxQueue);connected=true;}
    void onDisconnect(BLEServer *) override {connected=false;if(rxQueue)xQueueReset(rxQueue);restartAdvertising=true;}
};
class WriteCallbacks final:public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        const std::string value=characteristic->getValue();
        if(!rxQueue || value.size()>uxQueueSpacesAvailable(rxQueue)) {
            dropped.fetch_add(uint32_t(value.size()));return;
        }
        // Only enqueue. No parser, flash, motor bus or control state here.
        for(const char c:value) {
            const uint8_t byte=uint8_t(c);
            if(xQueueSend(rxQueue,&byte,0)!=pdTRUE)dropped.fetch_add(1);
        }
    }
};
ServerCallbacks serverCallbacks;
WriteCallbacks writeCallbacks;
}
bool beginBle(const char *name) {
    if(server)return ready;
    if(!name || !*name)return false;
    rxQueue=xQueueCreate(256,sizeof(uint8_t));if(!rxQueue)return false;
    // Arduino BLEDevice uses btStart() under ARDUINO_ARCH_ESP32. No NVS init,
    // bonding, erase or settings writes are performed by this module.
    BLEDevice::init(name);
    if(esp_bluedroid_get_status()!=ESP_BLUEDROID_STATUS_ENABLED)return false;
    BLEDevice::setMTU(64);
    server=BLEDevice::createServer();if(!server)return false;
    server->setCallbacks(&serverCallbacks);
    BLEService *service=server->createService(kService);if(!service)return false;
    tx=service->createCharacteristic(kNotify,BLECharacteristic::PROPERTY_NOTIFY);
    if(!tx)return false;
    tx->addDescriptor(new BLE2902());
    BLECharacteristic *rx=service->createCharacteristic(kWrite,BLECharacteristic::PROPERTY_WRITE);
    if(!rx)return false;
    rx->setCallbacks(&writeCallbacks);service->start();
    BLEAdvertising *advertising=server->getAdvertising();
    advertising->addServiceUUID(kService);advertising->setScanResponse(true);
    advertising->start();ready=true;return true;
}
size_t readBle(uint8_t *out,size_t capacity) {
    if(!rxQueue || !out)return 0;
    size_t n=0;
    while(n<capacity && xQueueReceive(rxQueue,out+n,0)==pdTRUE)++n;
    return n;
}
bool notifyBle(const uint8_t *bytes,size_t length) {
    if(!tx || !server || !connected || !bytes || !length)return false;
    const uint16_t mtu=server->getPeerMTU(server->getConnId());
    const size_t chunk=mtu>3?size_t(mtu-3):20;
    // Serial-over-BLE response can be reassembled as the same frame stream.
    for(size_t offset=0;offset<length;offset+=chunk) {
        if(!connected)return false;
        const size_t count=std::min(chunk,length-offset);
        tx->setValue(const_cast<uint8_t *>(bytes+offset),count);tx->notify();
    }
    return true;
}
bool bleConnected(){return connected.load();}
uint32_t bleDroppedBytes(){return dropped.load();}
void pollBle(){if(restartAdvertising.exchange(false) && server)server->startAdvertising();}
}
