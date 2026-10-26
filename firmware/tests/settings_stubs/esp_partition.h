#pragma once
#include <stdint.h>
struct esp_partition_t {uint32_t address,size;};
using esp_partition_iterator_t=const esp_partition_t *;
constexpr int ESP_PARTITION_TYPE_ANY=255,ESP_PARTITION_SUBTYPE_ANY=255;
esp_partition_iterator_t esp_partition_find(int,int,const char *);
const esp_partition_t *esp_partition_get(esp_partition_iterator_t);
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t);
void esp_partition_iterator_release(esp_partition_iterator_t);
