#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MicroRosManager.h"

extern "C" void app_main(void) {
    printf("app_main started\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    MicroRosManager micro_ros;

    if (!micro_ros.begin()) {
        ESP_LOGE("app_main", "MicroRosManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (true) {
        micro_ros.update();

        if (micro_ros.hasNewJointCommand()) {
            double joints[MicroRosManager::MAX_JOINTS];
            size_t size = 0;
            micro_ros.consumeJointCommand(joints, size);

            printf("Consumed command: ");
            for (size_t i = 0; i < size; ++i) {
                printf("%.6f ", joints[i]);
            }
            printf("\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}