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

static const char *TAG = "app_main";

////// SPI0 test code

// extern "C" void app_main(void)
// {
//     printf("SPI0 TEST START\n");

//     SPI0Manager spi0;

//     if (!spi0.begin()) {
//         printf("SPI0 init failed\n");
//         while (true) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }

//     printf("SPI0 initialized\n");

//     printf("=== MCP DEBUG ===\n");

//     spi0.debugReadMcpRegisters(UI_1_HBUTTONS);
//     spi0.debugReadMcpRegisters(UI_2_LEDS_AND_SWITCHES);
//     spi0.debugReadMcpRegisters(UI_3_VBUTTONS);
//     spi0.debugReadMcpRegisters(MAINVALVE);

//     printf("=== END MCP DEBUG ===\n");

//     // You can keep or comment the LED loop for now
//     while (true) {

//         {
//             uint16_t hButtons = spi0.readHorizontalButtons();
//             uint16_t vButtons = spi0.readVerticalButtons();
//             uint8_t switches  = spi0.readSwitches();

//             printf("H Buttons: 0x%04X | V Buttons: 0x%04X | Switches: 0x%02X\n",
//                 hButtons, vButtons, switches);

//             vTaskDelay(pdMS_TO_TICKS(500));

//             // LED TEST
//             static uint8_t ledState = 0;
//             spi0.debugWriteAndReadLeds(ledState);
//             ledState = (ledState + 1) % 64; // cycle through LED states 
//         }

//     }

// }



/////// EMERGENCY BUTTON TEST CODE



// #define EMERGENCY_PIN 32

// extern "C" void app_main(void)
// {
//     gpio_config_t io_conf = {};
//     io_conf.pin_bit_mask = (1ULL << EMERGENCY_PIN);
//     io_conf.mode = GPIO_MODE_INPUT;
//     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     // try this first
//     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     io_conf.intr_type = GPIO_INTR_DISABLE;

//     gpio_config(&io_conf);

//     printf("Emergency button test started on GPIO %d\n", EMERGENCY_PIN);

//     int last = -1;

//     while (true) {
//         int level = gpio_get_level((gpio_num_t)EMERGENCY_PIN);

//         if (level != last) {
//             printf("Emergency pin level: %d\n", level);
//             last = level;
//         }

//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

extern "C" void app_main(void) {
    printf("app_main started\n");

    // ============================================================
    // Static objects to avoid stack overflow
    // ============================================================
    static MicroRosManager micro_ros;
    static RobotController robot_controller;
    //static InputController input_controller(InputModes::JOINT_TARGET_MODE);

    static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

    size_t path_waypoints = 0;
    size_t path_dof = 0;
    size_t current_wp = 0;
    bool executing_path = false;

    TickType_t waypoint_start_tick = 0;
    const TickType_t waypoint_hold_ticks = pdMS_TO_TICKS(1000);

    // ============================================================
    // Initialize NVS
    // ============================================================
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ============================================================
    // Initialize controllers
    // ============================================================
    robot_controller.begin();
    //input_controller.begin();

    //input_controller.webserverAttachRobotController(robot_controller);
    //input_controller.beginWebserver();

    // ============================================================
    // Initialize micro-ROS
    // ============================================================
    if (!micro_ros.begin()) {
        ESP_LOGE(TAG, "MicroRosManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Application initialized successfully");

    // ============================================================
    // Main loop
    // ============================================================
    while (true) {
        micro_ros.update();

        if (micro_ros.hasNewPath()) {
            micro_ros.consumePath(path, path_waypoints, path_dof);
            current_wp = 0;
            executing_path = (path_waypoints > 0);
            waypoint_start_tick = xTaskGetTickCount();

            ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
        }

        if (executing_path && current_wp < path_waypoints) {
            std::vector<float> target;
            target.reserve(path_dof);

            for (size_t i = 0; i < path_dof; ++i) {
                target.push_back(static_cast<float>(path[current_wp][i]));
            }

            robot_controller.setJointTargetRad(target);

            TickType_t now = xTaskGetTickCount();
            if ((now - waypoint_start_tick) >= waypoint_hold_ticks) {
                ESP_LOGI(TAG, "Advancing from waypoint %d / %d",
                         (int)(current_wp + 1), (int)path_waypoints);

                current_wp++;
                waypoint_start_tick = now;

                if (current_wp >= path_waypoints) {
                    executing_path = false;
                    ESP_LOGI(TAG, "Path execution finished");
                }
            }
        }

        //input_controller.update(robot_controller);
        robot_controller.update();
        robot_controller.service();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}