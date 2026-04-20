#pragma once

#include <stddef.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/string.h>

class MicroRosManager {
public:
    static constexpr size_t MAX_JOINTS = 5;
    static constexpr size_t MAX_WAYPOINTS = 30;
    static constexpr size_t MAX_PATH_MSG_VALUES = 2 + MAX_WAYPOINTS * MAX_JOINTS;

    static constexpr size_t MAX_ESP_CMD_LEN = 64;

    MicroRosManager();

    bool begin();
    void update();

    bool hasNewPath() const;
    void consumePath(double out[][MAX_JOINTS], size_t &waypoints, size_t &dof);

    bool hasNewEspCmd() const;
    void consumeEspCmd(char *out, size_t out_size);

private:
    static void jointPathCallback(const void *msgin);
    static void espCmdCallback(const void *msgin);
    static MicroRosManager *instance_;

    bool createEntities();
    void destroyEntities();
    void zeroInitRosObjects();

private:
    bool started_ = false;
    bool ros_entities_created_ = false;
    bool new_path_available_ = false;
    bool new_esp_cmd_available_ = false;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;

    // Subscriber: path
    rcl_subscription_t subscriber_;

    // Subscriber: ESP command
    rcl_subscription_t esp_cmd_subscriber_;

    rclc_executor_t executor_;

    // joint path
    std_msgs__msg__Float64MultiArray msg_;
    double msg_buffer_[MAX_PATH_MSG_VALUES];

    // esp cmd string
    std_msgs__msg__String esp_cmd_msg_;
    char esp_cmd_buffer_[MAX_ESP_CMD_LEN];

    double latest_path_[MAX_WAYPOINTS][MAX_JOINTS];
    size_t latest_path_waypoints_ = 0;
    size_t latest_path_dof_ = 0;

    // latest raw ESP command
    char latest_esp_cmd_[MAX_ESP_CMD_LEN];
};