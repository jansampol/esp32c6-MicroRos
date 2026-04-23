#include <stdio.h>
#include <vector>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "SystemParameters.h"
#include "MicroRosController/MicroRosManager.h"
#include "RobotController/RobotController.h"
#include "InputController/SPI/SPI1Manager.h"
#include "InputController/InputController.h"
#include "pinDefinitions.h"

static const char *TAG = "app_main";

namespace
{
    bool parseLong(const char *text, long &value)
    {
        if (text == nullptr || *text == '\0') {
            return false;
        }

        char *endptr = nullptr;
        value = strtol(text, &endptr, 10);

        return (endptr != text && *endptr == '\0');
    }

    void handleEspCommand(const char *cmd, RobotController &robot_controller)
    {
        if (cmd == nullptr || *cmd == '\0') {
            ESP_LOGW(TAG, "Received empty ESP command");
            return;
        }

        ESP_LOGI(TAG, "Handling ESP command: %s", cmd);

        // Global command
        if (strcmp(cmd, "home_all") == 0) {
            robot_controller.sendAllJointsToHome();
            ESP_LOGI(TAG, "Applied command: home_all");
            return;
        }

        // Expected format:
        //   <jointIdx>:zero
        //   <jointIdx>:home
        //   <jointIdx>:<deltaSteps>
        const char *sep = strchr(cmd, ':');
        if (sep == nullptr) {
            ESP_LOGW(TAG, "Invalid ESP cmd format (missing ':'): %s", cmd);
            return;
        }

        char left[16] = {0};
        char right[32] = {0};

        size_t left_len = static_cast<size_t>(sep - cmd);
        if (left_len == 0 || left_len >= sizeof(left)) {
            ESP_LOGW(TAG, "Invalid joint field in cmd: %s", cmd);
            return;
        }

        strncpy(left, cmd, left_len);
        left[left_len] = '\0';

        strncpy(right, sep + 1, sizeof(right) - 1);
        right[sizeof(right) - 1] = '\0';

        long joint_idx_long = 0;
        if (!parseLong(left, joint_idx_long)) {
            ESP_LOGW(TAG, "Invalid joint index in cmd: %s", cmd);
            return;
        }

        if (joint_idx_long < 0 ||
            joint_idx_long >= static_cast<long>(MicroRosManager::MAX_JOINTS)) {
            ESP_LOGW(TAG, "Joint index out of range in cmd: %s", cmd);
            return;
        }

        const size_t joint_idx = static_cast<size_t>(joint_idx_long);

        if (strcmp(right, "zero") == 0) {
            robot_controller.tareJointToZero(joint_idx);
            ESP_LOGI(TAG, "Applied command: joint[%d] zero", (int)joint_idx);
            return;
        }

        if (strcmp(right, "home") == 0) {
            robot_controller.sendJointToHome(joint_idx);
            ESP_LOGI(TAG, "Applied command: joint[%d] home", (int)joint_idx);
            return;
        }

        long delta_long = 0;
        if (!parseLong(right, delta_long)) {
            ESP_LOGW(TAG, "Invalid joint delta/action in cmd: %s", cmd);
            return;
        }

        robot_controller.jogJointSteps(joint_idx, static_cast<int>(delta_long));
        ESP_LOGI(
            TAG,
            "Applied command: joint[%d] jog %+d",
            (int)joint_idx,
            (int)delta_long
        );
    }
}

extern "C" void app_main(void) {
    printf("app_main started\n");

    // ============================================================
    // Initialize NVS
    // ============================================================
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    size_t path_waypoints = 0;
    size_t path_dof = 0;
    size_t current_wp = 0;
    bool executing_path = false;
    bool waypoint_sent = false;

    static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];


    #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
    // ============================================================
    // SPI0 mode: full integration path (micro-ROS + robot + input)
    // ============================================================
    static MicroRosManager micro_ros;
    static RobotController robot_controller;
    static InputController input_controller(InputModes::JOINT_TARGET_MODE);

    robot_controller.begin();
    input_controller.begin();
    // input_controller.webserverAttachRobotController(robot_controller);
    // input_controller.beginWebserver();

    if (!micro_ros.begin()) {
        ESP_LOGE(TAG, "MicroRosManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    bool mainValveOn = true;
    input_controller.setMainValve(mainValveOn);
    ESP_LOGI(TAG, "Application initialized successfully (SPI0 mode)");

    while (true) {
        micro_ros.update();

        if (micro_ros.hasNewEspCmd()) {
            char cmd[64];
            micro_ros.consumeEspCmd(cmd, sizeof(cmd));
            executing_path = false;
            waypoint_sent = false;
            handleEspCommand(cmd, robot_controller);
        }

        if (micro_ros.hasNewPath()) {
            micro_ros.consumePath(path, path_waypoints, path_dof);
            current_wp = 0;
            executing_path = (path_waypoints > 0);
            waypoint_sent = false;
            ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
        }

        if (executing_path && current_wp < path_waypoints) {
            if (!waypoint_sent) {
                std::vector<float> target;
                target.reserve(path_dof);
                for (size_t i = 0; i < path_dof; ++i) {
                    target.push_back(static_cast<float>(path[current_wp][i]));
                }
                robot_controller.setJointTargetRad(target);
                waypoint_sent = true;

                ESP_LOGI(TAG, "Sent waypoint %d / %d",
                         (int)(current_wp + 1), (int)path_waypoints);
            }

            if (robot_controller.isAtStepTarget()) {
                ESP_LOGI(TAG, "Reached waypoint %d / %d",
                         (int)(current_wp + 1), (int)path_waypoints);
                current_wp++;
                waypoint_sent = false;

                if (current_wp >= path_waypoints) {
                    executing_path = false;
                    ESP_LOGI(TAG, "Path execution finished");
                }
            }
        }

        robot_controller.update();
        robot_controller.service();
        input_controller.update(robot_controller);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    #else
    // ============================================================
    // SPI1 mode: micro-ROS + robot loop
    // ============================================================
    static MicroRosManager micro_ros;
    static RobotController robot_controller;

    //size_t path_waypoints = 0;
    //size_t path_dof = 0;
    //size_t current_wp = 0;
    //bool executing_path = false;
    //TickType_t waypoint_start_tick = 0;
    //const TickType_t waypoint_hold_ticks = pdMS_TO_TICKS(1000);

    robot_controller.begin();

    if (!micro_ros.begin()) {
        ESP_LOGE(TAG, "MicroRosManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Application initialized successfully (SPI1 mode)");

    // while (true) {
    //     micro_ros.update();

    //     if (micro_ros.hasNewEspCmd()) {
    //         char cmd[64];
    //         micro_ros.consumeEspCmd(cmd, sizeof(cmd));
    //         executing_path = false;
    //         handleEspCommand(cmd, robot_controller);
    //     }

    //     if (micro_ros.hasNewPath()) {
    //         micro_ros.consumePath(path, path_waypoints, path_dof);
    //         current_wp = 0;
    //         executing_path = (path_waypoints > 0);
    //         waypoint_start_tick = xTaskGetTickCount();
    //         ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
    //     }

    //     if (executing_path && current_wp < path_waypoints) {
    //         std::vector<float> target;
    //         target.reserve(path_dof);
    //         for (size_t i = 0; i < path_dof; ++i) {
    //             target.push_back(static_cast<float>(path[current_wp][i]));
    //         }
    //         robot_controller.setJointTargetRad(target);

    //         TickType_t now = xTaskGetTickCount();
    //         if ((now - waypoint_start_tick) >= waypoint_hold_ticks) {
    //             ESP_LOGI(TAG, "Advancing from waypoint %d / %d",
    //                      (int)(current_wp + 1), (int)path_waypoints);
    //             current_wp++;
    //             waypoint_start_tick = now;
    //             if (current_wp >= path_waypoints) {
    //                 executing_path = false;
    //                 ESP_LOGI(TAG, "Path execution finished");
    //             }
    //         }
    //     }

    //     robot_controller.update();
    //     robot_controller.service();
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    while (true) {
        micro_ros.update();

        // ------------------------------------------------------------
        // Manual/debug ESP commands
        // ------------------------------------------------------------
        if (micro_ros.hasNewEspCmd()) {
            char cmd[64];
            micro_ros.consumeEspCmd(cmd, sizeof(cmd));

            // Stop automatic path execution when manual/debug command arrives
            executing_path = false;
            waypoint_sent = false;

            handleEspCommand(cmd, robot_controller);
        }

        // ------------------------------------------------------------
        // Planned path execution
        // ------------------------------------------------------------
        if (micro_ros.hasNewPath()) {
            micro_ros.consumePath(path, path_waypoints, path_dof);
            current_wp = 0;
            executing_path = (path_waypoints > 0);
            waypoint_sent = false;

            ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
        }

        if (executing_path && current_wp < path_waypoints) {
            if (!waypoint_sent) {
                std::vector<float> target;
                target.reserve(path_dof);

                for (size_t i = 0; i < path_dof; ++i) {
                    target.push_back(static_cast<float>(path[current_wp][i]));
                }

                robot_controller.setJointTargetRad(target);
                waypoint_sent = true;

                ESP_LOGI(TAG, "Sent waypoint %d / %d",
                         (int)(current_wp + 1),
                         (int)path_waypoints);
            }

            if (robot_controller.isAtStepTarget()) {
                ESP_LOGI(TAG, "Reached waypoint %d / %d",
                         (int)(current_wp + 1),
                         (int)path_waypoints);

                current_wp++;
                waypoint_sent = false;

                if (current_wp >= path_waypoints) {
                    executing_path = false;
                    ESP_LOGI(TAG, "Path execution finished");
                }
            }
        }

    robot_controller.update();
    robot_controller.service();
    vTaskDelay(pdMS_TO_TICKS(10));
    }
    #endif

}


// // extern "C" void app_main(void) {
// //     printf("app_main started\n");
// //     static RobotController robot_controller;

// //     // ============================================================
// //     // Initialize NVS
// //     // ============================================================
// //     esp_err_t ret = nvs_flash_init();
// //     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
// //         ESP_ERROR_CHECK(nvs_flash_erase());
// //         ret = nvs_flash_init();
// //     }
// //     ESP_ERROR_CHECK(ret);

// //     // ============================================================
// //     // Initialize controllers
// //     // ============================================================
// //     robot_controller.begin();

// //     #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
// //     static InputController input_controller(InputModes::JOINT_TARGET_MODE);
// //     input_controller.begin();
// //     ESP_LOGI(TAG, "Runtime mode: SPI0-only (InputController enabled, SPI1 valves disabled)");
// //     while (true) {
// //         input_controller.update(robot_controller);
// //         robot_controller.update();
// //         robot_controller.service();

// //         vTaskDelay(pdMS_TO_TICKS(10));
// //     }
// //     #else
// //     static MicroRosManager micro_ros;
// //     static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

// //     size_t path_waypoints = 0;
// //     size_t path_dof = 0;
// //     size_t current_wp = 0;
// //     bool executing_path = false;
// //     TickType_t waypoint_start_tick = 0;
// //     const TickType_t waypoint_hold_ticks = pdMS_TO_TICKS(1000);

// //     ESP_LOGI(TAG, "Runtime mode: SPI1-only (InputController disabled)");

// //     // ============================================================
// //     // Initialize micro-ROS
// //     // ============================================================
// //     if (!micro_ros.begin()) {
// //         ESP_LOGE(TAG, "MicroRosManager begin() failed");
// //         while (true) {
// //             vTaskDelay(pdMS_TO_TICKS(1000));
// //         }
// //     }

// //     ESP_LOGI(TAG, "Application initialized successfully");

// //     while (true) {
// //         micro_ros.update();

// //         if (micro_ros.hasNewEspCmd()) {
// //             char cmd[64];
// //             micro_ros.consumeEspCmd(cmd, sizeof(cmd));
// //             executing_path = false;
// //             handleEspCommand(cmd, robot_controller);
// //         }

// //         if (micro_ros.hasNewPath()) {
// //             micro_ros.consumePath(path, path_waypoints, path_dof);
// //             current_wp = 0;
// //             executing_path = (path_waypoints > 0);
// //             waypoint_start_tick = xTaskGetTickCount();
// //             ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
// //         }

// //         if (executing_path && current_wp < path_waypoints) {
// //             std::vector<float> target;
// //             target.reserve(path_dof);
// //             for (size_t i = 0; i < path_dof; ++i) {
// //                 target.push_back(static_cast<float>(path[current_wp][i]));
// //             }
// //             robot_controller.setJointTargetRad(target);

// //             TickType_t now = xTaskGetTickCount();
// //             if ((now - waypoint_start_tick) >= waypoint_hold_ticks) {
// //                 ESP_LOGI(TAG, "Advancing from waypoint %d / %d",
// //                          (int)(current_wp + 1), (int)path_waypoints);
// //                 current_wp++;
// //                 waypoint_start_tick = now;
// //                 if (current_wp >= path_waypoints) {
// //                     executing_path = false;
// //                     ESP_LOGI(TAG, "Path execution finished");
// //                 }
// //             }
// //         }

// //         robot_controller.update();
// //         robot_controller.service();
// //         vTaskDelay(pdMS_TO_TICKS(10));
// //     }
// //     #endif
// // }

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // Integration SPI0 + MicroRos + RobotController + InputController (NO VALVES)
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// // static const char* TAG = "app_main";

// // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // // Main application entry point
// // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // extern "C" void app_main(void)
// // {
// //     ESP_LOGI(TAG, "Starting runtime (InputController + RobotController), SPI1 disabled in RobotController");

// //     esp_err_t ret = nvs_flash_init();
// //     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
// //         ESP_ERROR_CHECK(nvs_flash_erase());
// //         ret = nvs_flash_init();
// //     }
// //     ESP_ERROR_CHECK(ret);

// //     static InputController input_controller(InputModes::JOINT_TARGET_MODE);
// //     static RobotController robot_controller;

// //     input_controller.begin();
// //     robot_controller.begin();

// //     ESP_LOGI(TAG, "Runtime initialized");

// //     bool mainValveOn = true;
// //     input_controller.setMainValve(mainValveOn);

// //     while (true) {
// //         robot_controller.update();
// //         robot_controller.service();
// //         input_controller.update(robot_controller);


// //         //toggle the mai valve every 4 loops
// //         // static uint8_t loopCount = 0;
// //         // if ((loopCount % 3) == 0) {
// //         //     mainValveOn = !mainValveOn;
// //         //     input_controller.setMainValve(mainValveOn);
// //         // }
// //         // loopCount++;

// //         vTaskDelay(pdMS_TO_TICKS(20));
// //     }
// // }



// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // extern "C" void app_main(void)
// // {
// //     ESP_LOGI(TAG, "Starting combined runtime test (SPI0 + I2C + UI, SPI1 disabled)");

// //     esp_err_t ret = nvs_flash_init();
// //     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
// //         ESP_ERROR_CHECK(nvs_flash_erase());
// //         ret = nvs_flash_init();
// //     }
// //     ESP_ERROR_CHECK(ret);

// //     static InputController input_controller(InputModes::JOINT_TARGET_MODE);
// //     static RobotController robot_controller;

// //     input_controller.begin();
// //     robot_controller.begin();

// //     ESP_LOGI(TAG, "Combined runtime initialized");

// //     bool mainValveOn = true;
// //     input_controller.setMainValve(mainValveOn);

// //     while (true) {
// //         robot_controller.update();
// //         robot_controller.service();
// //         input_controller.update(robot_controller);

// //         vTaskDelay(pdMS_TO_TICKS(20));
// //     }
// // }

// //////////////////////////////////////////////////////
// // I2C test code
// //////////////////////////////////////////////////////

// // extern "C" void app_main(void)
// // {
// //     printf("FERRIS + PRESSURE TEST START\n");

// //     // ============================================================
// //     // Initialize NVS
// //     // ============================================================
// //     esp_err_t ret = nvs_flash_init();
// //     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
// //         ESP_ERROR_CHECK(nvs_flash_erase());
// //         ret = nvs_flash_init();
// //     }
// //     ESP_ERROR_CHECK(ret);

// //     // ============================================================
// //     // Create I2C Manager
// //     // ============================================================
// //     static I2CManager i2cManager;

// //     uint8_t numFerris = 0;

// //     if (!i2cManager.begin(numFerris)) {
// //         ESP_LOGE(TAG, "I2CManager begin() failed");
// //         while (true) {
// //             vTaskDelay(pdMS_TO_TICKS(1000));
// //         }
// //     }

// //     ESP_LOGI(TAG, "I2C initialized. Starting Ferris + pressure test...");

// //     // spi0 manager
// //     SPI0Manager spi0;
// //     if (!spi0.begin()) {
// //         ESP_LOGE(TAG, "SPI0Manager begin() failed");
// //         while (true) {
// //             vTaskDelay(pdMS_TO_TICKS(1000));
// //         }
// //     }

// //     ESP_LOGI(TAG, "SPI0 initialized successfully");
    
// //     bool valveOpen = false;
// //     // Turn on the mauin valve
// //     spi0.setMainValve(valveOpen);


// //     while (true) {

// //         // Read ferris wheels
// //         // for (uint8_t i = 0; i < i2cManager.getNumOfFerrisWheels(); i++) {
// //         //     float raw = i2cManager.readFerrisWheelRawValue(i);
// //         //     float angle = i2cManager.readFerrisWheelAngle(i);

// //         //     ESP_LOGI(
// //         //         "FERRIS_TEST",
// //         //         "wheel=%u | raw=%.2f | angle=%.2f deg",
// //         //         i,
// //         //         raw,
// //         //         angle
// //         //     );
// //         // }

// //         // Read pressure sensor
// //         float pressure0 = i2cManager.readPressureSensor(1);

// //         if (isnan(pressure0)) {
// //             ESP_LOGE("PRESSURE_TEST", "pressure sensor 1 read failed");
// //         } else {
// //             ESP_LOGI("PRESSURE_TEST", "pressure sensor 1 = %.4f bar", pressure0);
// //         }

// //         ESP_LOGI(TAG, "--------------------------------------");

// //         //toggle the mai valve every 4 loops
// //         static uint8_t loopCount = 0;
// //         if ((loopCount % 4) == 0) {
// //             valveOpen = !valveOpen;
// //             spi0.setMainValve(valveOpen);
// //         }
// //         loopCount++;


// //         vTaskDelay(pdMS_TO_TICKS(500));
// //     }
// // }






// #include <stdio.h>
// #include <vector>
// #include <cstdint>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "nvs_flash.h"
// #include "esp_err.h"
// #include "esp_log.h"
// #include "driver/gpio.h"

// #include "MicroRosController/MicroRosManager.h"
// #include "RobotController/RobotController.h"
// //#include "InputController/InputController.h"
// #include "pinDefinitions.h"

// static const char *TAG = "app_main";



// namespace
// {
//     bool parseLong(const char *text, long &value)
//     {
//         if (text == nullptr || *text == '\0') {
//             return false;
//         }

//         char *endptr = nullptr;
//         value = strtol(text, &endptr, 10);

//         return (endptr != text && *endptr == '\0');
//     }

//     void handleEspCommand(const char *cmd, RobotController &robot_controller)
//     {
//         if (cmd == nullptr || *cmd == '\0') {
//             ESP_LOGW(TAG, "Received empty ESP command");
//             return;
//         }

//         ESP_LOGI(TAG, "Handling ESP command: %s", cmd);

//         // Global command
//         if (strcmp(cmd, "home_all") == 0) {
//             robot_controller.sendAllJointsToHome();
//             ESP_LOGI(TAG, "Applied command: home_all");
//             return;
//         }

//         // Expected format:
//         //   <jointIdx>:zero
//         //   <jointIdx>:home
//         //   <jointIdx>:<deltaSteps>
//         const char *sep = strchr(cmd, ':');
//         if (sep == nullptr) {
//             ESP_LOGW(TAG, "Invalid ESP cmd format (missing ':'): %s", cmd);
//             return;
//         }

//         char left[16] = {0};
//         char right[32] = {0};

//         size_t left_len = static_cast<size_t>(sep - cmd);
//         if (left_len == 0 || left_len >= sizeof(left)) {
//             ESP_LOGW(TAG, "Invalid joint field in cmd: %s", cmd);
//             return;
//         }

//         strncpy(left, cmd, left_len);
//         left[left_len] = '\0';

//         strncpy(right, sep + 1, sizeof(right) - 1);
//         right[sizeof(right) - 1] = '\0';

//         long joint_idx_long = 0;
//         if (!parseLong(left, joint_idx_long)) {
//             ESP_LOGW(TAG, "Invalid joint index in cmd: %s", cmd);
//             return;
//         }

//         if (joint_idx_long < 0 ||
//             joint_idx_long >= static_cast<long>(MicroRosManager::MAX_JOINTS)) {
//             ESP_LOGW(TAG, "Joint index out of range in cmd: %s", cmd);
//             return;
//         }

//         const size_t joint_idx = static_cast<size_t>(joint_idx_long);

//         if (strcmp(right, "zero") == 0) {
//             robot_controller.tareJointToZero(joint_idx);
//             ESP_LOGI(TAG, "Applied command: joint[%d] zero", (int)joint_idx);
//             return;
//         }

//         if (strcmp(right, "home") == 0) {
//             robot_controller.sendJointToHome(joint_idx);
//             ESP_LOGI(TAG, "Applied command: joint[%d] home", (int)joint_idx);
//             return;
//         }

//         long delta_long = 0;
//         if (!parseLong(right, delta_long)) {
//             ESP_LOGW(TAG, "Invalid joint delta/action in cmd: %s", cmd);
//             return;
//         }

//         robot_controller.jogJointSteps(joint_idx, static_cast<int>(delta_long));
//         ESP_LOGI(
//             TAG,
//             "Applied command: joint[%d] jog %+d",
//             (int)joint_idx,
//             (int)delta_long
//         );
//     }
// }

// extern "C" void app_main(void) {
//     printf("app_main started\n");

//     // ============================================================
//     // Static objects to avoid stack overflow
//     // ============================================================
//     static MicroRosManager micro_ros;
//     static RobotController robot_controller;
//     // static InputController input_controller(InputModes::JOINT_TARGET_MODE);

//     static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

//     size_t path_waypoints = 0;
//     size_t path_dof = 0;
//     size_t current_wp = 0;
//     bool executing_path = false;
//     bool waypoint_sent = false;

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
//     // Initialize controllers
//     // ============================================================
//     robot_controller.begin();

//     // Safer low speed for manual debug / homing
//     // robot_controller.setFrequency(3.0f);

//     // input_controller.begin();
//     // input_controller.webserverAttachRobotController(robot_controller);
//     // input_controller.beginWebserver();

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

//         // ------------------------------------------------------------
//         // Manual/debug ESP commands
//         // ------------------------------------------------------------
//         if (micro_ros.hasNewEspCmd()) {
//             char cmd[64];
//             micro_ros.consumeEspCmd(cmd, sizeof(cmd));

//             // Stop automatic path execution when manual/debug command arrives
//             executing_path = false;
//             waypoint_sent = false;

//             handleEspCommand(cmd, robot_controller);
//         }

//         // ------------------------------------------------------------
//         // Planned path execution
//         // ------------------------------------------------------------
//         if (micro_ros.hasNewPath()) {
//             micro_ros.consumePath(path, path_waypoints, path_dof);
//             current_wp = 0;
//             executing_path = (path_waypoints > 0);
//             waypoint_sent = false;

//             ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
//         }

//         if (executing_path && current_wp < path_waypoints) {
//             if (!waypoint_sent) {
//                 std::vector<float> target;
//                 target.reserve(path_dof);

//                 for (size_t i = 0; i < path_dof; ++i) {
//                     target.push_back(static_cast<float>(path[current_wp][i]));
//                 }

//                 robot_controller.setJointTargetRad(target);
//                 waypoint_sent = true;

//                 ESP_LOGI(TAG, "Sent waypoint %d / %d",
//                          (int)(current_wp + 1),
//                          (int)path_waypoints);
//             }

//             if (robot_controller.isAtStepTarget()) {
//                 ESP_LOGI(TAG, "Reached waypoint %d / %d",
//                          (int)(current_wp + 1),
//                          (int)path_waypoints);

//                 current_wp++;
//                 waypoint_sent = false;

//                 if (current_wp >= path_waypoints) {
//                     executing_path = false;
//                     ESP_LOGI(TAG, "Path execution finished");
//                 }
//             }
//         }

//         // ------------------------------------------------------------
//         // Robot low-level update/service
//         // ------------------------------------------------------------
//         // input_controller.update(robot_controller);
//         robot_controller.update();
//         robot_controller.service();

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// extern "C" void app_main(void) {
//     printf("app_main started\n");

//     // ============================================================
//     // Static objects to avoid stack overflow
//     // ============================================================
//     static MicroRosManager micro_ros;
//     static RobotController robot_controller;
//     // static InputController input_controller(InputModes::JOINT_TARGET_MODE);

//     static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

//     size_t path_waypoints = 0;
//     size_t path_dof = 0;
//     //size_t current_wp = 0;
//     //bool executing_path = false;

//     TickType_t waypoint_start_tick = 0;
//     const TickType_t waypoint_hold_ticks = pdMS_TO_TICKS(1000);

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
//     // Initialize controllers
//     // ============================================================
//     robot_controller.begin();

//     // Safer low speed for manual debug / homing
//     //robot_controller.setFrequency(3.0f);

//     // input_controller.begin();
//     // input_controller.webserverAttachRobotController(robot_controller);
//     // input_controller.beginWebserver();

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

//     size_t current_wp = 0;
//     bool executing_path = false;
//     bool waypoint_sent = false;

//     while (true) {
//         micro_ros.update();

//         if (micro_ros.hasNewPath()) {
//             micro_ros.consumePath(path, path_waypoints, path_dof);
//             current_wp = 0;
//             executing_path = (path_waypoints > 0);
//             waypoint_sent = false;

//             ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
//         }

//         if (executing_path && current_wp < path_waypoints) {
//             if (!waypoint_sent) {
//                 std::vector<float> target;
//                 target.reserve(path_dof);

//                 for (size_t i = 0; i < path_dof; ++i) {
//                     target.push_back(static_cast<float>(path[current_wp][i]));
//                 }

//                 robot_controller.setJointTargetRad(target);
//                 waypoint_sent = true;

//                 ESP_LOGI(TAG, "Sent waypoint %d / %d",
//                         (int)(current_wp + 1),
//                         (int)path_waypoints);
//             }

//             if (robot_controller.isAtStepTarget()) {
//                 ESP_LOGI(TAG, "Reached waypoint %d / %d",
//                         (int)(current_wp + 1),
//                         (int)path_waypoints);

//                 current_wp++;
//                 waypoint_sent = false;

//                 if (current_wp >= path_waypoints) {
//                     executing_path = false;
//                     ESP_LOGI(TAG, "Path execution finished");
//                 }
//             }
//         }

//         robot_controller.update();
//         robot_controller.service();

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

    // while (true) {
    //     micro_ros.update();

    //     // ------------------------------------------------------------
    //     // Manual/debug ESP commands
    //     // ------------------------------------------------------------
    //     if (micro_ros.hasNewEspCmd()) {
    //         char cmd[64];
    //         micro_ros.consumeEspCmd(cmd, sizeof(cmd));

    //         // Stop automatic path execution when manual/debug command arrives
    //         executing_path = false;

    //         handleEspCommand(cmd, robot_controller);
    //     }

    //     // ------------------------------------------------------------
    //     // Planned path execution
    //     // ------------------------------------------------------------
    //     if (micro_ros.hasNewPath()) {
    //         micro_ros.consumePath(path, path_waypoints, path_dof);
    //         current_wp = 0;
    //         executing_path = (path_waypoints > 0);
    //         waypoint_start_tick = xTaskGetTickCount();

    //         ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
    //     }

    //     if (executing_path && current_wp < path_waypoints) {
    //         std::vector<float> target;
    //         target.reserve(path_dof);

    //         for (size_t i = 0; i < path_dof; ++i) {
    //             target.push_back(static_cast<float>(path[current_wp][i]));
    //         }

    //         robot_controller.setJointTargetRad(target);

    //         TickType_t now = xTaskGetTickCount();
    //         if ((now - waypoint_start_tick) >= waypoint_hold_ticks) {
    //             ESP_LOGI(
    //                 TAG,
    //                 "Advancing from waypoint %d / %d",
    //                 (int)(current_wp + 1),
    //                 (int)path_waypoints
    //             );

    //             current_wp++;
    //             waypoint_start_tick = now;

    //             if (current_wp >= path_waypoints) {
    //                 executing_path = false;
    //                 ESP_LOGI(TAG, "Path execution finished");
    //             }
    //         }
    //    }

        // ------------------------------------------------------------
        // Robot low-level update/service
        // ------------------------------------------------------------
        // input_controller.update(robot_controller);
    //     robot_controller.update();
    //     robot_controller.service();

    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
//}







///////

// extern "C" void app_main(void) {
//     printf("app_main started\n");

//     // ============================================================
//     // Static objects to avoid stack overflow
//     // ============================================================
//     static MicroRosManager micro_ros;
//     static RobotController robot_controller;
//     // static InputController input_controller(InputModes::JOINT_TARGET_MODE);

//     static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

//     size_t path_waypoints = 0;
//     size_t path_dof = 0;
//     size_t current_wp = 0;
//     bool executing_path = false;
//     bool waypoint_sent = false;

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
//     // Initialize controllers
//     // ============================================================
//     robot_controller.begin();

//     // Safer low speed for manual debug / homing
//     // robot_controller.setFrequency(3.0f);

//     // input_controller.begin();
//     // input_controller.webserverAttachRobotController(robot_controller);
//     // input_controller.beginWebserver();

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

//         // ------------------------------------------------------------
//         // Manual/debug ESP commands
//         // ------------------------------------------------------------
//         if (micro_ros.hasNewEspCmd()) {
//             char cmd[64];
//             micro_ros.consumeEspCmd(cmd, sizeof(cmd));

//             // Stop automatic path execution when manual/debug command arrives
//             executing_path = false;
//             waypoint_sent = false;

//             handleEspCommand(cmd, robot_controller);
//         }

//         // ------------------------------------------------------------
//         // Planned path execution
//         // ------------------------------------------------------------
//         if (micro_ros.hasNewPath()) {
//             micro_ros.consumePath(path, path_waypoints, path_dof);
//             current_wp = 0;
//             executing_path = (path_waypoints > 0);
//             waypoint_sent = false;

//             ESP_LOGI(TAG, "Received path with %d waypoints", (int)path_waypoints);
//         }

//         if (executing_path && current_wp < path_waypoints) {
//             if (!waypoint_sent) {
//                 std::vector<float> target;
//                 target.reserve(path_dof);

//                 for (size_t i = 0; i < path_dof; ++i) {
//                     target.push_back(static_cast<float>(path[current_wp][i]));
//                 }

//                 robot_controller.setJointTargetRad(target);
//                 waypoint_sent = true;

//                 ESP_LOGI(TAG, "Sent waypoint %d / %d",
//                          (int)(current_wp + 1),
//                          (int)path_waypoints);
//             }

//             if (robot_controller.isAtStepTarget()) {
//                 ESP_LOGI(TAG, "Reached waypoint %d / %d",
//                          (int)(current_wp + 1),
//                          (int)path_waypoints);

//                 current_wp++;
//                 waypoint_sent = false;

//                 if (current_wp >= path_waypoints) {
//                     executing_path = false;
//                     ESP_LOGI(TAG, "Path execution finished");
//                 }
//             }
//         }

//         // ------------------------------------------------------------
//         // Robot low-level update/service
//         // ------------------------------------------------------------
//         // input_controller.update(robot_controller);
//         robot_controller.update();
//         robot_controller.service();

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }