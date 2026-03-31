#pragma once

#include <stddef.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

class MicroRosManager {
public:
    static constexpr size_t MAX_JOINTS = 10;

    MicroRosManager();

    bool begin();
    void update();

    bool hasNewJointCommand() const;
    void consumeJointCommand(double *out, size_t &size);

private:
    static void jointAnglesCallback(const void *msgin);
    static MicroRosManager *instance_;

    bool createEntities();
    void destroyEntities();
    void zeroInitRosObjects();

private:
    bool started_ = false;
    bool ros_entities_created_ = false;
    bool new_joint_command_available_ = false;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rcl_subscription_t subscriber_;
    rclc_executor_t executor_;

    std_msgs__msg__Float64MultiArray msg_;
    double msg_buffer_[MAX_JOINTS];

    double latest_joint_command_[MAX_JOINTS];
    size_t latest_joint_command_size_ = 0;
};