#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "SystemParameters.h"
#include "MicroRosController/MicroRosManager.h"
#include "RobotController/RobotController.h"
#include "InputController/I2C/I2CManager.h"
#include "InputController/InputController.h"

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

    void handleEspCommand(const char *cmd, RobotController &robot_controller, bool &incision_mode)
    {
        if (cmd == nullptr || *cmd == '\0') {
            ESP_LOGW(TAG, "Received empty ESP command");
            return;
        }

        ESP_LOGI(TAG, "Handling ESP command: %s", cmd);

        if (strcmp(cmd, "incision_on") == 0) {
            incision_mode = true;
            for (int i = 0; i < robot_controller.getNumOfSteppers(); ++i) {
                robot_controller.setTargetVelocity(static_cast<size_t>(i), 0.0f);
            }
            robot_controller.setControlStrategy(PneumaticStepper::Controlstrategy::VELOCITY_CONTROL);
            ESP_LOGI(TAG, "Applied command: incision_on");
            return;
        }

        if (strcmp(cmd, "incision_off") == 0) {
            incision_mode = false;
            for (int i = 0; i < robot_controller.getNumOfSteppers(); ++i) {
                robot_controller.setTargetVelocity(static_cast<size_t>(i), 0.0f);
            }
            const int numSteppers = robot_controller.getNumOfSteppers();
            if (numSteppers > 0) {
                const size_t needleJointIdx = static_cast<size_t>(numSteppers - 1);
                const RobotState state = robot_controller.getRobotState();
                if (needleJointIdx < state.jointSteps.size()) {
                    robot_controller.setJointTargetStep(needleJointIdx, state.jointSteps[needleJointIdx]);
                }
            }
            robot_controller.setControlStrategy(PneumaticStepper::Controlstrategy::POSITION_CONTROL);
            ESP_LOGI(TAG, "Applied command: incision_off");
            return;
        }

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
            joint_idx_long >= static_cast<long>(robot_controller.getNumOfSteppers())) {
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

extern "C" void app_main(void)
{
    printf("app_main started\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    size_t path_waypoints = 0;
    size_t path_dof = 0;
    bool path_has_insertion_depth = false;
    float path_insertion_depth_mm = 0.0f;
    bool needle_target_available = false;
    int needle_target_steps = 0;
#if ACTIVE_SPI_RUNTIME_MODE != SPI_RUNTIME_MODE_SPI0_ONLY
    (void)needle_target_available;
    (void)needle_target_steps;
#endif
    size_t current_wp = 0;
    bool executing_path = false;
    bool waypoint_sent = false;
    bool incision_mode = false;

    static double path[MicroRosManager::MAX_WAYPOINTS][MicroRosManager::MAX_JOINTS];

    static RobotController robot_controller;
    static MicroRosManager micro_ros;

    #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
    static InputController input_controller(InputModes::JOINT_TARGET_MODE);
    input_controller.begin();
    #endif

    robot_controller.begin();

    if (!micro_ros.begin()) {
        ESP_LOGE(TAG, "MicroRosManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
    static I2CManager i2cManager;
    uint8_t numFerris = 5;
    if (!i2cManager.begin(numFerris)) {
        ESP_LOGE(TAG, "I2CManager begin() failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    #endif

    #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
    input_controller.setMainValve(true);
    ESP_LOGI(TAG, "Application initialized successfully (SPI0 input mode)");
    #else
    ESP_LOGI(TAG, "Application initialized successfully (SPI1 motor mode)");
    #endif

    while (true) {
        micro_ros.update();

        if (micro_ros.hasNewEspCmd()) {
            char cmd[64];
            micro_ros.consumeEspCmd(cmd, sizeof(cmd));
            executing_path = false;
            waypoint_sent = false;
            handleEspCommand(cmd, robot_controller, incision_mode);
        }

        if (micro_ros.hasNewPath()) {
            micro_ros.consumePath(
                path,
                path_waypoints,
                path_dof,
                path_has_insertion_depth,
                path_insertion_depth_mm
            );
            current_wp = 0;
            executing_path = (path_waypoints > 0) && !incision_mode;
            waypoint_sent = false;
            if (path_has_insertion_depth) {
                needle_target_steps = robot_controller.needleDepthMmToSteps(path_insertion_depth_mm);
                needle_target_available = true;
                ESP_LOGI(TAG,
                         "Stored needle incision target: %.3f mm -> %d steps",
                         path_insertion_depth_mm,
                         needle_target_steps);
            } else {
                needle_target_available = false;
                needle_target_steps = 0;
            }
            ESP_LOGI(TAG,
                     "Received path with %d waypoints%s%s",
                     (int)path_waypoints,
                     path_has_insertion_depth ? " and insertion depth" : "",
                     incision_mode ? " (ignored while incision mode is active)" : "");
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
                    if (!micro_ros.publishRobotState("movement_finished")) {
                        ESP_LOGW(TAG, "Failed to publish movement_finished");
                    }
                }
            }
        }

        //#if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
        //float pressure1 = i2cManager.readPressureSensor(1);
        //float pressure0 = i2cManager.readPressureSensor(0);

        // uint16_t valve_state = robot_controller.getValveState();
        // uint8_t j1_a = (valve_state >> 0) & 0x1;
        // uint8_t j1_b = (valve_state >> 1) & 0x1;

        // if (std::isnan(pressure0) || std::isnan(pressure1)) {
        //     ESP_LOGE("PRESSURE_TEST", "pressure read failed p0=%.4f p1=%.4f", pressure0, pressure1);
        // } else {
        //     ESP_LOGI("DATA_CSV", "%llu,%.4f,%.4f,%u,%u,0x%04X",
        //              (unsigned long long)(esp_timer_get_time() / 1000ULL),
        //              pressure1,
        //              pressure0,
        //              j1_a,
        //              j1_b,
        //              valve_state);
        //}
        //#endif

        // Read ferris wheel angles for logging and potential use in control (not currently used in control).
        // #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
        // static TickType_t lastFerrisLogTick = 0;
        // const TickType_t now = xTaskGetTickCount();
        // if ((now - lastFerrisLogTick) >= pdMS_TO_TICKS(500)) {
        //     lastFerrisLogTick = now;

        //     std::vector<float> ferris_angles = i2cManager.readAllFerrisWheelAngles();
        //     std::vector<float> ferris_raw = i2cManager.readAllFerrisWheelRawValues();
        //     robot_controller.setFerrisWheelFeedback(ferris_angles);

        //     for (size_t i = 0; i < ferris_angles.size(); ++i) {
        //         const float raw = (i < ferris_raw.size()) ? ferris_raw[i] : NAN;
        //         ESP_LOGI(TAG,
        //                  "Ferris wheel %d: angle=%.2f deg raw=%.2f",
        //                  (int)i,
        //                  ferris_angles[i],
        //                  raw);
        //     }
        // }
        // #endif  

        robot_controller.update();
        robot_controller.service();

        #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
        input_controller.update(
            robot_controller,
            incision_mode,
            needle_target_available,
            needle_target_steps
        );
        #endif

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
