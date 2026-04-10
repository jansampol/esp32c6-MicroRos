#include <stdio.h>
#include <vector>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "MicroRosController/MicroRosManager.h"
#include "RobotController/RobotController.h"
#include "InputController/InputController.h"
//#include "InputController/SPI/SPI0Manager.h"

//static const char *TAG = "app_main";

extern "C" void app_main(void)
{
    printf("SPI0 TEST START\n");

    SPI0Manager spi0;

    if (!spi0.begin()) {
        printf("SPI0 init failed\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("SPI0 initialized\n");

    while (true) {
        // active-high test
        for (int i = 0; i < 6; i++) {
            uint8_t pattern = (1 << i);
            printf("ACTIVE-HIGH pattern: 0x%02X\n", pattern);
            spi0.debugWriteAndReadLeds(pattern);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // all off / all on candidates
        printf("Pattern 0x00\n");
        spi0.debugWriteAndReadLeds(0x00);
        vTaskDelay(pdMS_TO_TICKS(1000));

        printf("Pattern 0x3F\n");
        spi0.debugWriteAndReadLeds(0x3F);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // active-low test
        for (int i = 0; i < 6; i++) {
            uint8_t pattern = (~(1 << i)) & 0x3F;
            printf("ACTIVE-LOW pattern: 0x%02X\n", pattern);
            spi0.debugWriteAndReadLeds(pattern);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// extern "C" void app_main(void) {
//     printf("app_main started\n");

//     // ============================================================
//     // Initialize NVS
//     // ============================================================
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // ============================================================
//     // Create main application objects
//     // ============================================================
//     MicroRosManager micro_ros;
//     RobotController robot_controller;
//     InputController input_controller(InputModes::JOINT_TARGET_MODE);

//     // ============================================================
//     // Initialize controllers
//     // ============================================================
//     robot_controller.begin();
//     input_controller.begin();

//     input_controller.webserverAttachRobotController(robot_controller);
//     input_controller.beginWebserver();

//     // ============================================================
//     // Initialize micro-ROS
//     // ============================================================
//     if (!micro_ros.begin()) {
//         ESP_LOGE(TAG, "MicroRosManager begin() failed");
//         while (true) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }

//     ESP_LOGI(TAG, "Application initialized successfully");

//     // ============================================================
//     // Main loop
//     // ============================================================
//     while (true) {
//         micro_ros.update();

//         if (micro_ros.hasNewJointCommand()) {
//             double joints[MicroRosManager::MAX_JOINTS];
//             size_t size = 0;

//             micro_ros.consumeJointCommand(joints, size);

//             std::vector<float> joint_angles;
//             joint_angles.reserve(size);

//             for (size_t i = 0; i < size; ++i) {
//                 joint_angles.push_back(static_cast<float>(joints[i]));
//             }

//             ESP_LOGI(TAG, "Passing joint command to RobotController");
//             robot_controller.setJointTargetRad(joint_angles);
//         }

//         input_controller.update(robot_controller);

//         robot_controller.update();
//         robot_controller.service();

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }