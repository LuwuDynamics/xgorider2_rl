#pragma once
#include <stddef.h>
#include <stdint.h>
constexpr int ESP_OK=0;
int spi_flash_read(uint32_t address,void *out,size_t length);
int spi_flash_write(uint32_t address,const void *data,size_t length);
int spi_flash_erase_sector(size_t sector);
