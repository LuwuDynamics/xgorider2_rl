#include <esp_err.h>
#include <nvs_flash.h>

// Full BLE firmware needs a real NVS service. Permit normal initialization, but
// prevent Arduino 2.0.17 from automatically formatting the entire NVS partition
// after an incompatibility/full-pages error. No forced erase or migration here.
extern "C" esp_err_t __real_nvs_flash_init(void);
extern "C" esp_err_t __wrap_nvs_flash_init(void) {
    const esp_err_t result=__real_nvs_flash_init();
    if(result==ESP_ERR_NVS_NO_FREE_PAGES || result==ESP_ERR_NVS_NEW_VERSION_FOUND)return ESP_FAIL;
    return result;
}

// Also defer Arduino's automatic OTA confirmation if a future SDK build enables
// rollback support. The uploader accepts only stable/undefined OTA states.
extern "C" bool verifyRollbackLater() { return true; }
