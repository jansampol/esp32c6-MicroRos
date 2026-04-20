#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "RobotController/RobotController.h"
#include "InputController/InputController.h"

static const char* TAG = "app_main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting runtime (InputController + RobotController), SPI1 disabled in RobotController");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    static InputController input_controller(InputModes::JOINT_TARGET_MODE);
    static RobotController robot_controller;

    input_controller.begin();
    robot_controller.begin();

    ESP_LOGI(TAG, "Runtime initialized");

    bool mainValveOn = false;
    uint32_t valveTick = 0;

    while (true) {
        robot_controller.update();
        robot_controller.service();
        input_controller.update(robot_controller);

        valveTick += 20;
        if (valveTick >= 1000) {
            valveTick = 0;
            mainValveOn = !mainValveOn;
            input_controller.setMainValve(mainValveOn);
            ESP_LOGI(TAG, "Main valve: %s", mainValveOn ? "ON" : "OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
