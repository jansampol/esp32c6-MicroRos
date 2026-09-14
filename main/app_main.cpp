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

    bool isCurrentJointPositionCommand(const char *cmd)
    {
        if (cmd == nullptr) {
            return false;
        }
        return strcmp(cmd, "get_init_pos") == 0 ||
               strcmp(cmd, "get_current_joint_pos") == 0;
    }

    bool publishCurrentJointPosition(RobotController &robot_controller, MicroRosManager &micro_ros)
    {
        const RobotState state = robot_controller.getRobotState();
        const size_t dof = static_cast<size_t>(robot_controller.getDegreesOfFreedom());

        if (dof == 0 ||
            state.ferrisWheelJointSteps.size() < dof ||
            state.ferrisWheelRawValues.size() < dof ||
            !state.needsPositionalFeedback) {
            ESP_LOGW(TAG, "Cannot publish initial position: Ferris feedback is not ready");
            return micro_ros.publishRobotState("init_pos_rad,0");
        }

        const std::vector<int> ferrisSteps(
            state.ferrisWheelJointSteps.begin(),
            state.ferrisWheelJointSteps.begin() + dof
        );
        const std::vector<float> jointRad = robot_controller.stepsToRad(ferrisSteps);

        char payload[MicroRosManager::MAX_ESP_CMD_LEN] = {0};
        int written = snprintf(payload, sizeof(payload), "init_pos_rad,1");
        for (size_t i = 0; i < jointRad.size() && written > 0 && written < static_cast<int>(sizeof(payload)); ++i) {
            written += snprintf(
                payload + written,
                sizeof(payload) - static_cast<size_t>(written),
                ",%.6f",
                static_cast<double>(jointRad[i])
            );
        }

        if (written < 0 || written >= static_cast<int>(sizeof(payload))) {
            ESP_LOGW(TAG, "Initial position payload was truncated");
            payload[sizeof(payload) - 1] = '\0';
        }

        ESP_LOGI(TAG, "Publishing Ferris initial position: %s", payload);
        return micro_ros.publishRobotState(payload);
    }

    void handleEspCommand(
        const char *cmd,
        RobotController &robot_controller,
        MicroRosManager &micro_ros,
        bool &incision_mode,
        bool needle_target_available,
        int needle_target_steps
    )
    {
        if (cmd == nullptr || *cmd == '\0') {
            ESP_LOGW(TAG, "Received empty ESP command");
            return;
        }

        ESP_LOGI(TAG, "Handling ESP command: %s", cmd);

        if (strcmp(cmd, "incision_on") == 0) {
            incision_mode = true;
            const RobotState state = robot_controller.getRobotState();
            for (int i = 0; i < robot_controller.getNumOfSteppers(); ++i) {
                robot_controller.setTargetVelocity(static_cast<size_t>(i), 0.0f);
                if (static_cast<size_t>(i) < state.jointSteps.size()) {
                    robot_controller.setJointTargetStep(static_cast<size_t>(i), state.jointSteps[i]);
                }
            }
            robot_controller.setControlStrategy(PneumaticStepper::Controlstrategy::POSITION_CONTROL);
            robot_controller.setNeedleVelocityControlEnabled(true);
            ESP_LOGI(TAG, "Applied command: incision_on");
            return;
        }

        if (strcmp(cmd, "incision_off") == 0) {
            incision_mode = false;
            robot_controller.setNeedleVelocityControlEnabled(false);
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

        if (strcmp(cmd,"reset_robot") == 0) {
            incision_mode = false;
            robot_controller.setNeedleVelocityControlEnabled(false);
            ESP_LOGI(TAG, "Applied command: reset_robot");
            return;
        }

        if (isCurrentJointPositionCommand(cmd)) {
            if (!publishCurrentJointPosition(robot_controller, micro_ros)) {
                ESP_LOGW(TAG, "Failed to publish initial joint position");
            }
            ESP_LOGI(TAG, "Applied command: %s", cmd);
            return;
        }

        if (strcmp(cmd, "home_all") == 0) {
            robot_controller.sendAllJointsToHome();
            ESP_LOGI(TAG, "Applied command: home_all");
            return;
        }

        if (strcmp(cmd, "ferris_tare") == 0) {
            robot_controller.ferrisWheelTareCurrentPosition();
            ESP_LOGI(TAG, "Applied command: ferris:tare");
            return;
        }

        // Virtual needle slider command. It uses the same raw range and
        // velocity mapping as the physical slider.
        constexpr const char *needleSliderPrefix = "needle_slider:";
        constexpr size_t needleSliderPrefixLen = 14;
        if (strncmp(cmd, needleSliderPrefix, needleSliderPrefixLen) == 0) {
            long raw_slider_value = 0;
            if (!parseLong(cmd + needleSliderPrefixLen, raw_slider_value) ||
                raw_slider_value < 0 || raw_slider_value > 1023) {
                ESP_LOGW(TAG, "Invalid virtual needle slider command: %s", cmd);
                return;
            }

            if (!incision_mode) {
                ESP_LOGW(
                    TAG,
                    "Ignored virtual needle slider outside incision mode: raw=%ld",
                    raw_slider_value
                );
                return;
            }

            const int numSteppers = robot_controller.getNumOfSteppers();
            if (numSteppers <= 0) {
                ESP_LOGW(TAG, "Cannot apply virtual needle slider: no steppers configured");
                return;
            }

            constexpr float kVirtualNeedleMaxVelocity = 10.0f;
            const size_t needleJointIdx = static_cast<size_t>(numSteppers - 1);
            float needleVelocity = mapNeedleSliderToVelocity(
                static_cast<uint16_t>(raw_slider_value),
                kVirtualNeedleMaxVelocity
            );

            bool stoppedAtInsertionLimit = false;
            if (needle_target_available) {
                const RobotState state = robot_controller.getRobotState();
                if (needleJointIdx < state.jointSteps.size()) {
                    const int currentSteps = state.jointSteps[needleJointIdx];
                    if (needleVelocity > 0.0f && currentSteps >= needle_target_steps) {
                        needleVelocity = 0.0f;
                        stoppedAtInsertionLimit = true;
                    }
                }
            }

            for (int i = 0; i < numSteppers; ++i) {
                robot_controller.setTargetVelocity(static_cast<size_t>(i), 0.0f);
            }
            robot_controller.setFrequency(kVirtualNeedleMaxVelocity);
            robot_controller.setNeedleVelocityControlEnabled(true);
            robot_controller.setTargetVelocity(needleJointIdx, needleVelocity);

            ESP_LOGI(
                TAG,
                "Applied virtual needle slider: raw=%ld velocity=%.2f steps/s joint=%u",
                raw_slider_value,
                static_cast<double>(needleVelocity),
                (unsigned)needleJointIdx
            );
            if (stoppedAtInsertionLimit) {
                ESP_LOGI(
                    TAG,
                    "Virtual needle stopped at insertion limit: target=%d steps",
                    needle_target_steps
                );
            }
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
    bool path_is_incision_correction = false;
    bool needle_target_available = false;
    int needle_target_steps = 0;
    //uint32_t open_loop_log_counter = 0;
#if ACTIVE_SPI_RUNTIME_MODE != SPI_RUNTIME_MODE_SPI0_ONLY
    (void)needle_target_available;
    (void)needle_target_steps;
#endif
    //size_t current_wp = 0;
    bool executing_path = false;
    //bool waypoint_sent = false;
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
            char cmd[MicroRosManager::MAX_ESP_CMD_LEN];
            micro_ros.consumeEspCmd(cmd, sizeof(cmd));
            if (!isCurrentJointPositionCommand(cmd)) {
                executing_path = false;
                //waypoint_sent = false;
            }
            handleEspCommand(
                cmd,
                robot_controller,
                micro_ros,
                incision_mode,
                needle_target_available,
                needle_target_steps
            );
        }

        if (micro_ros.hasNewPath()) {
            micro_ros.consumePath(
                path,
                path_waypoints,
                path_dof,
                path_has_insertion_depth,
                path_insertion_depth_mm,
                path_is_incision_correction
            );
            // Convert C-style array to std::vector<std::vector<float>>
            std::vector<std::vector<float>> pathVec(path_waypoints);
            for (size_t i = 0; i < path_waypoints; ++i) {
                pathVec[i].assign(path[i], path[i] + path_dof);
            }
            robot_controller.setNewPath(pathVec, path_waypoints, path_dof);
            //current_wp = 0;
            const bool path_allowed_in_incision_mode = path_is_incision_correction;
            const bool suppress_motion_for_incision_mode = incision_mode && !path_allowed_in_incision_mode;
            executing_path = (path_waypoints > 0) && !suppress_motion_for_incision_mode;
            //waypoint_sent = false;
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
                     "Received path with %d waypoints%s%s%s",
                     (int)path_waypoints,
                     path_has_insertion_depth ? " and insertion depth" : "",
                     path_is_incision_correction ? " (incision correction)" : "",
                     suppress_motion_for_incision_mode ? " (motion suppressed while incision mode is active)" : "");
        }

        // if (executing_path && current_wp < path_waypoints) {
        //     if (!waypoint_sent) {
        //         std::vector<float> target;
        //         target.reserve(path_dof);
        //         for (size_t i = 0; i < path_dof; ++i) {
        //             target.push_back(static_cast<float>(path[current_wp][i]));
        //         }
        //         robot_controller.setJointTargetRad(target);
        //         waypoint_sent = true;

        //         ESP_LOGI(TAG, "Sent waypoint %d / %d",
        //                  (int)(current_wp + 1), (int)path_waypoints);
        //     }

        //     if (robot_controller.isAtStepTarget()) {
        //         ESP_LOGI(TAG, "Reached waypoint %d / %d",
        //                  (int)(current_wp + 1), (int)path_waypoints);
        //         current_wp++;
        //         waypoint_sent = false;

        //         if (current_wp >= path_waypoints) {
        //             executing_path = false;
        //             ESP_LOGI(TAG, "Path execution finished");
        //             if (!micro_ros.publishRobotState("movement_finished")) {
        //                 ESP_LOGW(TAG, "Failed to publish movement_finished");
        //             }
        //         }
        //     }
        // }

        // Read ferris wheel angles for logging and control.
        #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY

            std::vector<float> ferris_angles = i2cManager.readAllFerrisWheelAngles();
            std::vector<float> ferris_raw = i2cManager.readAllFerrisWheelAbsoluteRawValues();
            robot_controller.setFerrisWheelFeedback(ferris_angles, ferris_raw);
            robot_controller.processMotionControl(executing_path, path_waypoints, path_dof);
            
            // Check if path execution finished
            if (executing_path && !robot_controller.isPathExecuting()) {
                executing_path = false;
                if (!micro_ros.publishRobotState("movement_finished")) {
                    ESP_LOGW(TAG, "Failed to publish movement_finished");
                }
            }
        #endif  

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

















// #include <cstdint>
// #include <cstdio>
// #include <cstdlib>
// #include <cstring>
// #include <vector>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "nvs_flash.h"
// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_timer.h"

// #include "SystemParameters.h"
// #include "MicroRosController/MicroRosManager.h"
// #include "RobotController/RobotController.h"
// #include "InputController/I2C/I2CManager.h"
// #include "InputController/InputController.h"

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

//     bool isCurrentJointPositionCommand(const char *cmd)
//     {
//         if (cmd == nullptr) {
//             return false;
//         }
//         return strcmp(cmd, "get_init_pos") == 0 ||
//                strcmp(cmd, "get_current_joint_pos") == 0;
//     }

//     bool publishCurrentJointPosition(RobotController &robot_controller, MicroRosManager &micro_ros)
//     {
//         const RobotState state = robot_controller.getRobotState();
//         const size_t dof = static_cast<size_t>(robot_controller.getDegreesOfFreedom());

//         if (dof == 0 ||
//             state.ferrisWheelJointSteps.size() < dof ||
//             state.ferrisWheelRawValues.size() < dof ||
//             !state.needsPositionalFeedback) {
//             ESP_LOGW(TAG, "Cannot publish initial position: Ferris feedback is not ready");
//             return micro_ros.publishRobotState("init_pos_rad,0");
//         }

//         const std::vector<int> ferrisSteps(
//             state.ferrisWheelJointSteps.begin(),
//             state.ferrisWheelJointSteps.begin() + dof
//         );
//         const std::vector<float> jointRad = robot_controller.stepsToRad(ferrisSteps);

//         char payload[MicroRosManager::MAX_ESP_CMD_LEN] = {0};
//         int written = snprintf(payload, sizeof(payload), "init_pos_rad,1");
//         for (size_t i = 0; i < jointRad.size() && written > 0 && written < static_cast<int>(sizeof(payload)); ++i) {
//             written += snprintf(
//                 payload + written,
//                 sizeof(payload) - static_cast<size_t>(written),
//                 ",%.6f",
//                 static_cast<double>(jointRad[i])
//             );
//         }

//         if (written < 0 || written >= static_cast<int>(sizeof(payload))) {
//             ESP_LOGW(TAG, "Initial position payload was truncated");
//             payload[sizeof(payload) - 1] = '\0';
//         }

//         ESP_LOGI(TAG, "Publishing Ferris initial position: %s", payload);
//         return micro_ros.publishRobotState(payload);
//     }

//     void handleEspCommand(
//         const char *cmd,
//         RobotController &robot_controller,
//         MicroRosManager &micro_ros,
//         bool &incision_mode
//     )
//     {
//         if (cmd == nullptr || *cmd == '\0') {
//             ESP_LOGW(TAG, "Received empty ESP command");
//             return;
//         }

//         ESP_LOGI(TAG, "Handling ESP command: %s", cmd);

//         if (strcmp(cmd, "incision_on") == 0) {
//             incision_mode = true;

//             for (int i = 0; i < robot_controller.getNumOfSteppers(); ++i) {
//                 robot_controller.setTargetVelocity(
//                     static_cast<size_t>(i),
//                     0.0f
//                 );
//             }

//             robot_controller.setControlStrategy(
//                 PneumaticStepper::Controlstrategy::VELOCITY_CONTROL
//             );

//             ESP_LOGI(TAG, "Applied command: incision_on");
//             return;
//         }

//         if (strcmp(cmd, "incision_off") == 0) {
//             incision_mode = false;

//             for (int i = 0; i < robot_controller.getNumOfSteppers(); ++i) {
//                 robot_controller.setTargetVelocity(
//                     static_cast<size_t>(i),
//                     0.0f
//                 );
//             }

//             const int numSteppers = robot_controller.getNumOfSteppers();

//             if (numSteppers > 0) {
//                 const size_t needleJointIdx =
//                     static_cast<size_t>(numSteppers - 1);

//                 const RobotState state = robot_controller.getRobotState();

//                 if (needleJointIdx < state.jointSteps.size()) {
//                     robot_controller.setJointTargetStep(
//                         needleJointIdx,
//                         state.jointSteps[needleJointIdx]
//                     );
//                 }
//             }

//             robot_controller.setControlStrategy(
//                 PneumaticStepper::Controlstrategy::POSITION_CONTROL
//             );

//             ESP_LOGI(TAG, "Applied command: incision_off");
//             return;
//         }

//         if (strcmp(cmd, "home_all") == 0) {
//             const std::vector<float> trial_home_rad = {
//                 0.4f,
//                 0.0f,
//                 0.0f,
//                 0.0f,
//                -0.4f
//             };

//             robot_controller.setJointTargetRad(trial_home_rad);
//             ESP_LOGI(
//                 TAG,
//                 "Applied command: home_all -> [0.4, 0.0, 0.0, 0.0, -0.4] rad"
//             );
//             return;
//         }

//         if (strcmp(cmd, "get_init_pos") == 0 ||
//             strcmp(cmd, "get_current_joint_pos") == 0) {
//             if (!publishCurrentJointPosition(robot_controller, micro_ros)) {
//                 ESP_LOGW(TAG, "Failed to publish current joint position");
//             }
//             ESP_LOGI(TAG, "Applied command: %s", cmd);
//             return;
//         }

//         // Expected format:
//         //   <jointIdx>:zero
//         //   <jointIdx>:home
//         //   <jointIdx>:<deltaSteps>
//         const char *sep = strchr(cmd, ':');

//         if (sep == nullptr) {
//             ESP_LOGW(
//                 TAG,
//                 "Invalid ESP cmd format (missing ':'): %s",
//                 cmd
//             );
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

//         if (
//             joint_idx_long < 0 ||
//             joint_idx_long >=
//                 static_cast<long>(robot_controller.getNumOfSteppers())
//         ) {
//             ESP_LOGW(TAG, "Joint index out of range in cmd: %s", cmd);
//             return;
//         }

//         const size_t joint_idx = static_cast<size_t>(joint_idx_long);

//         if (strcmp(right, "zero") == 0) {
//             robot_controller.tareJointToZero(joint_idx);

//             ESP_LOGI(
//                 TAG,
//                 "Applied command: joint[%d] zero",
//                 (int)joint_idx
//             );

//             return;
//         }

//         if (strcmp(right, "home") == 0) {
//             robot_controller.sendJointToHome(joint_idx);

//             ESP_LOGI(
//                 TAG,
//                 "Applied command: joint[%d] home",
//                 (int)joint_idx
//             );

//             return;
//         }

//         long delta_long = 0;

//         if (!parseLong(right, delta_long)) {
//             ESP_LOGW(
//                 TAG,
//                 "Invalid joint delta/action in cmd: %s",
//                 cmd
//             );
//             return;
//         }

//         robot_controller.jogJointSteps(
//             joint_idx,
//             static_cast<int>(delta_long)
//         );

//         ESP_LOGI(
//             TAG,
//             "Applied command: joint[%d] jog %+d",
//             (int)joint_idx,
//             (int)delta_long
//         );
//     }
// }

// extern "C" void app_main(void)
// {
//     printf("app_main started\n");

//     esp_err_t ret = nvs_flash_init();

//     if (
//         ret == ESP_ERR_NVS_NO_FREE_PAGES ||
//         ret == ESP_ERR_NVS_NEW_VERSION_FOUND
//     ) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }

//     ESP_ERROR_CHECK(ret);

//     size_t path_waypoints = 0;
//     size_t path_dof = 0;

//     bool path_has_insertion_depth = false;
//     float path_insertion_depth_mm = 0.0f;
//     bool path_is_incision_correction = false;

//     bool needle_target_available = false;
//     int needle_target_steps = 0;

// #if ACTIVE_SPI_RUNTIME_MODE != SPI_RUNTIME_MODE_SPI0_ONLY
//     (void)needle_target_available;
//     (void)needle_target_steps;
// #endif

//     size_t current_wp = 0;
//     bool executing_path = false;
//     bool waypoint_sent = false;
//     bool incision_mode = false;

//     static double path
//         [MicroRosManager::MAX_WAYPOINTS]
//         [MicroRosManager::MAX_JOINTS];

//     static RobotController robot_controller;
//     static MicroRosManager micro_ros;

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
//     static InputController input_controller(
//         InputModes::JOINT_TARGET_MODE
//     );

//     input_controller.begin();
// #endif

//     robot_controller.begin();

//     // The robot is physically placed in this configuration before startup.
//     // Seed the open-loop estimate and target with that pose so the first path
//     // is calculated from the real starting configuration, without moving.
//     const std::vector<float> initial_joint_rad = {
//         0.4f,
//         0.0f,
//         0.0f,
//         0.0f,
//        -0.4f
//     };
//     const std::vector<int> initial_joint_steps =
//         robot_controller.radToSteps(initial_joint_rad);
//     StepperPositions initial_stepper_positions =
//         robot_controller.getStepperPositions();

//     for (size_t i = 0;
//          i < initial_joint_steps.size() && i < 8;
//          ++i) {
//         initial_stepper_positions.jointSteps[i] = initial_joint_steps[i];
//     }

//     robot_controller.setStepperPositions(initial_stepper_positions);

//     ESP_LOGI(
//         TAG,
//         "Initialized OL joint estimate to [0.4, 0.0, 0.0, 0.0, -0.4] rad"
//     );

//     if (!micro_ros.begin()) {
//         ESP_LOGE(TAG, "MicroRosManager begin() failed");

//         while (true) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
//     static I2CManager i2cManager;
//     uint8_t numFerris = 5;

//     if (!i2cManager.begin(numFerris)) {
//         ESP_LOGE(TAG, "I2CManager begin() failed");

//         while (true) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }
// #endif

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
//     input_controller.setMainValve(true);

//     ESP_LOGI(
//         TAG,
//         "Application initialized successfully (SPI0 input mode)"
//     );
// #else
//     ESP_LOGI(
//         TAG,
//         "Application initialized successfully (SPI1 motor mode)"
//     );
// #endif

//     while (true) {
//         micro_ros.update();

//         if (micro_ros.hasNewEspCmd()) {
//             char cmd[MicroRosManager::MAX_ESP_CMD_LEN];

//             micro_ros.consumeEspCmd(cmd, sizeof(cmd));

//             if (!isCurrentJointPositionCommand(cmd)) {
//                 executing_path = false;
//                 waypoint_sent = false;
//             }

//             handleEspCommand(
//                 cmd,
//                 robot_controller,
//                 micro_ros,
//                 incision_mode
//             );
//         }

//         if (micro_ros.hasNewPath()) {
//             micro_ros.consumePath(
//                 path,
//                 path_waypoints,
//                 path_dof,
//                 path_has_insertion_depth,
//                 path_insertion_depth_mm,
//                 path_is_incision_correction
//             );

//             current_wp = 0;
//             executing_path =
//                 (path_waypoints > 0) && !incision_mode;
//             waypoint_sent = false;

//             if (path_has_insertion_depth) {
//                 needle_target_steps =
//                     robot_controller.needleDepthMmToSteps(
//                         path_insertion_depth_mm
//                     );

//                 needle_target_available = true;

//                 ESP_LOGI(
//                     TAG,
//                     "Stored needle incision target: %.3f mm -> %d steps",
//                     path_insertion_depth_mm,
//                     needle_target_steps
//                 );
//             } else {
//                 needle_target_available = false;
//                 needle_target_steps = 0;
//             }

//             ESP_LOGI(
//                 TAG,
//                 "Received path with %d waypoints%s%s",
//                 (int)path_waypoints,
//                 path_has_insertion_depth
//                     ? " and insertion depth"
//                     : "",
//                 incision_mode
//                     ? " (ignored while incision mode is active)"
//                     : ""
//             );
//         }

//         /*
//          * Open-loop waypoint controller.
//          *
//          * The controller sends joint targets and determines waypoint
//          * completion using the stepper's internally estimated positions.
//          * Ferris-wheel measurements are not used for correction.
//          */
//         if (executing_path && current_wp < path_waypoints) {
//             if (!waypoint_sent) {
//                 std::vector<float> target;
//                 target.reserve(path_dof);

//                 for (size_t i = 0; i < path_dof; ++i) {
//                     target.push_back(
//                         static_cast<float>(path[current_wp][i])
//                     );
//                 }

//                 robot_controller.setJointTargetRad(target);
//                 waypoint_sent = true;

//                 ESP_LOGI(
//                     TAG,
//                     "Sent waypoint %d / %d",
//                     (int)(current_wp + 1),
//                     (int)path_waypoints
//                 );
//             }

//             if (robot_controller.isAtStepTarget()) {
//                 ESP_LOGI(
//                     TAG,
//                     "Reached waypoint %d / %d",
//                     (int)(current_wp + 1),
//                     (int)path_waypoints
//                 );

//                 current_wp++;
//                 waypoint_sent = false;

//                 if (current_wp >= path_waypoints) {
//                     executing_path = false;

//                     ESP_LOGI(
//                         TAG,
//                         "Path execution finished"
//                     );

//                     if (
//                         !micro_ros.publishRobotState(
//                             "movement_finished"
//                         )
//                     ) {
//                         ESP_LOGW(
//                             TAG,
//                             "Failed to publish movement_finished"
//                         );
//                     }
//                 }
//             }
//         }

//         //#if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
//         //float pressure1 = i2cManager.readPressureSensor(1);
//         //float pressure0 = i2cManager.readPressureSensor(0);

//         // uint16_t valve_state = robot_controller.getValveState();
//         // uint8_t j1_a = (valve_state >> 0) & 0x1;
//         // uint8_t j1_b = (valve_state >> 1) & 0x1;

//         // if (std::isnan(pressure0) || std::isnan(pressure1)) {
//         //     ESP_LOGE(
//         //         "PRESSURE_TEST",
//         //         "pressure read failed p0=%.4f p1=%.4f",
//         //         pressure0,
//         //         pressure1
//         //     );
//         // } else {
//         //     ESP_LOGI(
//         //         "DATA_CSV",
//         //         "%llu,%.4f,%.4f,%u,%u,0x%04X",
//         //         (unsigned long long)(
//         //             esp_timer_get_time() / 1000ULL
//         //         ),
//         //         pressure1,
//         //         pressure0,
//         //         j1_a,
//         //         j1_b,
//         //         valve_state
//         //     );
//         //}
//         //#endif

//         /*
//          * Ferris-wheel readings were disabled in this version.
//          * They were not used for motion control.
//          */

//         // #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
//         // static TickType_t lastFerrisLogTick = 0;
//         // const TickType_t now = xTaskGetTickCount();

//         // if (
//         //     (now - lastFerrisLogTick) >=
//         //     pdMS_TO_TICKS(500)
//         // ) {
//         //     lastFerrisLogTick = now;

//         //     std::vector<float> ferris_angles =
//         //         i2cManager.readAllFerrisWheelAngles();

//         //     std::vector<float> ferris_raw =
//         //         i2cManager.readAllFerrisWheelRawValues();

//         //     robot_controller.setFerrisWheelFeedback(
//         //         ferris_angles
//         //     );

//         //     for (
//         //         size_t i = 0;
//         //         i < ferris_angles.size();
//         //         ++i
//         //     ) {
//         //         const float raw =
//         //             (i < ferris_raw.size())
//         //                 ? ferris_raw[i]
//         //                 : NAN;

//         //         ESP_LOGI(
//         //             TAG,
//         //             "Ferris wheel %d: angle=%.2f deg raw=%.2f",
//         //             (int)i,
//         //             ferris_angles[i],
//         //             raw
//         //         );
//         //     }
//         // }
//         // #endif

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
//         const std::vector<float> ferris_angles = i2cManager.readAllFerrisWheelAngles();
//         const std::vector<float> ferris_raw = i2cManager.readAllFerrisWheelAbsoluteRawValues();
//         robot_controller.setFerrisWheelFeedback(ferris_angles, ferris_raw);
// #endif

//         robot_controller.update();

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
//         static uint32_t ol_motion_log_counter = 0;
//         static uint32_t ol_motion_sample = 0;
//         ++ol_motion_log_counter;

//         if (ol_motion_log_counter >= 40) {
//             ol_motion_log_counter = 0;

//             const RobotState state = robot_controller.getRobotState();
//             const size_t dof = static_cast<size_t>(robot_controller.getDegreesOfFreedom());
//             const std::vector<float> open_loop_rad = robot_controller.stepsToRad(state.jointSteps);
//             const std::vector<float> command_rad = robot_controller.stepsToRad(state.targetJointSteps);
//             const std::vector<float> ferris_rad = robot_controller.stepsToRad(state.ferrisWheelJointSteps);
//             const uint64_t time_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
//             const uint32_t sample = ol_motion_sample++;

//             for (size_t i = 0; i < dof; ++i) {
//                 ESP_LOGI(
//                     TAG,
//                     "OL_MOTION_CSV: %llu,%u,%u,%d,%d,%.6f,%.6f,%d,%.6f,%.3f",
//                     (unsigned long long)time_ms,
//                     (unsigned)sample,
//                     (unsigned)i,
//                     (i < state.jointSteps.size()) ? state.jointSteps[i] : 0,
//                     (i < state.targetJointSteps.size()) ? state.targetJointSteps[i] : 0,
//                     (i < open_loop_rad.size()) ? static_cast<double>(open_loop_rad[i]) : 0.0,
//                     (i < command_rad.size()) ? static_cast<double>(command_rad[i]) : 0.0,
//                     (i < state.ferrisWheelJointSteps.size()) ? state.ferrisWheelJointSteps[i] : 0,
//                     (i < ferris_rad.size()) ? static_cast<double>(ferris_rad[i]) : 0.0,
//                     (i < state.ferrisWheelRawValues.size()) ? static_cast<double>(state.ferrisWheelRawValues[i]) : 0.0
//                 );
//             }
//         }
// #endif

//         robot_controller.service();

// #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI0_ONLY
//         input_controller.update(
//             robot_controller,
//             incision_mode,
//             needle_target_available,
//             needle_target_steps
//         );
// #endif

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
