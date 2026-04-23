#include "MicroRosManager.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <uros_network_interfaces.h>

static const char *TAG = "MicroRosManager";

MicroRosManager *MicroRosManager::instance_ = nullptr;

namespace
{
    void log_cleanup_rc(const char *name, rcl_ret_t rc)
    {
        if (rc != RCL_RET_OK) {
            ESP_LOGW(TAG, "%s failed during cleanup: %d", name, (int)rc);
        }
    }

    void log_rcl_error(const char *where, rcl_ret_t rc)
    {
        ESP_LOGE(TAG, "%s failed: rc=%d, msg=%s",
                 where,
                 (int)rc,
                 rcl_get_error_string().str);
        rcl_reset_error();
    }
}

MicroRosManager::MicroRosManager()
{
    zeroInitRosObjects();

    for (size_t i = 0; i < MAX_PATH_MSG_VALUES; ++i) {
        msg_buffer_[i] = 0.0;
    }

    for (size_t wp = 0; wp < MAX_WAYPOINTS; ++wp) {
        for (size_t j = 0; j < MAX_JOINTS; ++j) {
            latest_path_[wp][j] = 0.0;
        }
    }

    memset(&msg_, 0, sizeof(msg_));
    memset(&esp_cmd_msg_, 0, sizeof(esp_cmd_msg_));
    memset(esp_cmd_buffer_, 0, sizeof(esp_cmd_buffer_));
    memset(latest_esp_cmd_, 0, sizeof(latest_esp_cmd_));
}

void MicroRosManager::zeroInitRosObjects()
{
    memset(&support_, 0, sizeof(support_));
    node_ = rcl_get_zero_initialized_node();
    subscriber_ = rcl_get_zero_initialized_subscription();
    esp_cmd_subscriber_ = rcl_get_zero_initialized_subscription();
    executor_ = rclc_executor_get_zero_initialized_executor();
}

bool MicroRosManager::begin()
{
    instance_ = this;

    ESP_LOGI(TAG, "Initializing WiFi transport...");
    esp_err_t err = uros_network_interface_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uros_network_interface_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Waiting before micro-ROS init...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    if (!createEntities()) {
        ESP_LOGE(TAG, "createEntities() failed");
        return false;
    }

    started_ = true;
    ESP_LOGI(TAG, "micro-ROS base ready");
    return true;
}

bool MicroRosManager::createEntities()
{
    allocator_ = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    rcl_ret_t rc = rcl_init_options_init(&init_options, allocator_);
    if (rc != RCL_RET_OK) {
        log_rcl_error("rcl_init_options_init", rc);
        return false;
    }

    rc = rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rcl_init_options_get_rmw_init_options(&init_options)
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_options_set_udp_address failed: %d", (int)rc);
        rcl_ret_t rc_cleanup = rcl_init_options_fini(&init_options);
        if (rc_cleanup != RCL_RET_OK) {
            ESP_LOGW(TAG, "rcl_init_options_fini failed: %d", (int)rc_cleanup);
        }
        return false;
    }

    rc = rclc_support_init_with_options(
        &support_,
        0,
        NULL,
        &init_options,
        &allocator_
    );

    rcl_ret_t rc_cleanup = rcl_init_options_fini(&init_options);
    if (rc_cleanup != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_init_options_fini failed: %d", (int)rc_cleanup);
    }

    if (rc != RCL_RET_OK) {
        log_rcl_error("rclc_support_init_with_options", rc);
        return false;
    }

    rc = rclc_node_init_default(&node_, "esp32_path_subscriber", "", &support_);
    if (rc != RCL_RET_OK) {
        log_rcl_error("rclc_node_init_default", rc);

        rcl_ret_t rc2 = rclc_support_fini(&support_);
        if (rc2 != RCL_RET_OK) {
            ESP_LOGW(TAG, "rclc_support_fini failed: %d", (int)rc2);
        }

        zeroInitRosObjects();
        return false;
    }

    // -----------------------------
    // Path subscriber message setup
    // -----------------------------
    memset(&msg_, 0, sizeof(msg_));
    msg_.data.data = msg_buffer_;
    msg_.data.size = 0;
    msg_.data.capacity = MAX_PATH_MSG_VALUES;

    rc = rclc_subscription_init_default(
        &subscriber_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/mamri/planning/joint_path_execute"
    );
    if (rc != RCL_RET_OK) {
        log_rcl_error("path subscription init", rc);

        rcl_ret_t b = rcl_node_fini(&node_);
        rcl_ret_t c = rclc_support_fini(&support_);
        if (b != RCL_RET_OK) ESP_LOGW(TAG, "rcl_node_fini failed: %d", (int)b);
        if (c != RCL_RET_OK) ESP_LOGW(TAG, "rclc_support_fini failed: %d", (int)c);

        zeroInitRosObjects();
        return false;
    }

    // -----------------------------
    // ESP command subscriber message setup
    // -----------------------------
    memset(&esp_cmd_msg_, 0, sizeof(esp_cmd_msg_));
    memset(esp_cmd_buffer_, 0, sizeof(esp_cmd_buffer_));

    esp_cmd_msg_.data.data = esp_cmd_buffer_;
    esp_cmd_msg_.data.size = 0;
    esp_cmd_msg_.data.capacity = MAX_ESP_CMD_LEN;

    rc = rclc_subscription_init_default(
        &esp_cmd_subscriber_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/mamri/esp32/cmd"
    );
    if (rc != RCL_RET_OK) {
        log_rcl_error("esp cmd subscription init", rc);

        rcl_ret_t a = rcl_subscription_fini(&subscriber_, &node_);
        rcl_ret_t b = rcl_node_fini(&node_);
        rcl_ret_t c = rclc_support_fini(&support_);
        if (a != RCL_RET_OK) ESP_LOGW(TAG, "rcl_subscription_fini(path) failed: %d", (int)a);
        if (b != RCL_RET_OK) ESP_LOGW(TAG, "rcl_node_fini failed: %d", (int)b);
        if (c != RCL_RET_OK) ESP_LOGW(TAG, "rclc_support_fini failed: %d", (int)c);

        zeroInitRosObjects();
        return false;
    }

    rc = rclc_executor_init(&executor_, &support_.context, 2, &allocator_);
    if (rc != RCL_RET_OK) {
        log_rcl_error("rclc_executor_init", rc);

        rcl_ret_t a = rcl_subscription_fini(&esp_cmd_subscriber_, &node_);
        rcl_ret_t b = rcl_subscription_fini(&subscriber_, &node_);
        rcl_ret_t c = rcl_node_fini(&node_);
        rcl_ret_t d = rclc_support_fini(&support_);
        log_cleanup_rc("rcl_subscription_fini(esp_cmd)", a);
        log_cleanup_rc("rcl_subscription_fini(path)", b);
        log_cleanup_rc("rcl_node_fini", c);
        log_cleanup_rc("rclc_support_fini", d);

        zeroInitRosObjects();
        return false;
    }

    rc = rclc_executor_add_subscription(
        &executor_,
        &subscriber_,
        &msg_,
        &MicroRosManager::jointPathCallback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        log_rcl_error("rclc_executor_add_subscription(path)", rc);

        rcl_ret_t a = rclc_executor_fini(&executor_);
        rcl_ret_t b = rcl_subscription_fini(&esp_cmd_subscriber_, &node_);
        rcl_ret_t c = rcl_subscription_fini(&subscriber_, &node_);
        rcl_ret_t d = rcl_node_fini(&node_);
        rcl_ret_t e = rclc_support_fini(&support_);
        log_cleanup_rc("rclc_executor_fini", a);
        log_cleanup_rc("rcl_subscription_fini(esp_cmd)", b);
        log_cleanup_rc("rcl_subscription_fini(path)", c);
        log_cleanup_rc("rcl_node_fini", d);
        log_cleanup_rc("rclc_support_fini", e);

        zeroInitRosObjects();
        return false;
    }

    rc = rclc_executor_add_subscription(
        &executor_,
        &esp_cmd_subscriber_,
        &esp_cmd_msg_,
        &MicroRosManager::espCmdCallback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        log_rcl_error("rclc_executor_add_subscription(esp_cmd)", rc);

        rcl_ret_t a = rclc_executor_fini(&executor_);
        rcl_ret_t b = rcl_subscription_fini(&esp_cmd_subscriber_, &node_);
        rcl_ret_t c = rcl_subscription_fini(&subscriber_, &node_);
        rcl_ret_t d = rcl_node_fini(&node_);
        rcl_ret_t e = rclc_support_fini(&support_);
        log_cleanup_rc("rclc_executor_fini", a);
        log_cleanup_rc("rcl_subscription_fini(esp_cmd)", b);
        log_cleanup_rc("rcl_subscription_fini(path)", c);
        log_cleanup_rc("rcl_node_fini", d);
        log_cleanup_rc("rclc_support_fini", e);

        zeroInitRosObjects();
        return false;
    }

    ros_entities_created_ = true;
    ESP_LOGI(TAG, "micro-ROS entities created");
    return true;
}

void MicroRosManager::destroyEntities()
{
    if (!ros_entities_created_) {
        return;
    }

    rcl_ret_t rc1 = rclc_executor_fini(&executor_);
    rcl_ret_t rc2 = rcl_subscription_fini(&subscriber_, &node_);
    rcl_ret_t rc3 = rcl_subscription_fini(&esp_cmd_subscriber_, &node_);
    rcl_ret_t rc4 = rcl_node_fini(&node_);
    rcl_ret_t rc5 = rclc_support_fini(&support_);

    log_cleanup_rc("rclc_executor_fini", rc1);
    log_cleanup_rc("rcl_subscription_fini(path)", rc2);
    log_cleanup_rc("rcl_subscription_fini(esp_cmd)", rc3);
    log_cleanup_rc("rcl_node_fini", rc4);
    log_cleanup_rc("rclc_support_fini", rc5);

    ros_entities_created_ = false;
    zeroInitRosObjects();
}

void MicroRosManager::jointPathCallback(const void *msgin)
{
    printf("jointPathCallback called\n");
    if (instance_ == nullptr || msgin == nullptr) {
        return;
    }

    const auto *in = static_cast<const std_msgs__msg__Float64MultiArray *>(msgin);

    if (in->data.size < 2) {
        printf("Received invalid path message: too short\n");
        return;
    }

    size_t n_waypoints = static_cast<size_t>(in->data.data[0]);
    size_t dof = static_cast<size_t>(in->data.data[1]);

    if (dof != MAX_JOINTS) {
        printf("Received invalid path message: DOF=%zu expected=%zu\n", dof, MAX_JOINTS);
        return;
    }

    if (n_waypoints > MAX_WAYPOINTS) {
        printf("Received path truncated from %zu to %zu waypoints\n", n_waypoints, MAX_WAYPOINTS);
        n_waypoints = MAX_WAYPOINTS;
    }

    size_t expected_size = 2 + n_waypoints * dof;
    if (in->data.size < expected_size) {
        printf("Received invalid path message: expected at least %zu values, got %zu\n",
               expected_size, in->data.size);
        return;
    }

    size_t idx = 2;
    for (size_t wp = 0; wp < n_waypoints; ++wp) {
        for (size_t j = 0; j < dof; ++j) {
            instance_->latest_path_[wp][j] = in->data.data[idx++];
        }
    }

    instance_->latest_path_waypoints_ = n_waypoints;
    instance_->latest_path_dof_ = dof;
    instance_->new_path_available_ = true;

    printf("Received joint path: %zu waypoints, %zu dof\n", n_waypoints, dof);
}

void MicroRosManager::espCmdCallback(const void *msgin)
{
    printf("espCmdCallback called\n");
    if (instance_ == nullptr || msgin == nullptr) {
        return;
    }

    const auto *in = static_cast<const std_msgs__msg__String *>(msgin);

    if (in->data.data == nullptr || in->data.size == 0) {
        printf("Received empty ESP cmd\n");
        return;
    }

    size_t len = in->data.size;
    if (len >= MAX_ESP_CMD_LEN) {
        len = MAX_ESP_CMD_LEN - 1;
        printf("ESP cmd truncated to %zu chars\n", len);
    }

    memcpy(instance_->latest_esp_cmd_, in->data.data, len);
    instance_->latest_esp_cmd_[len] = '\0';
    instance_->new_esp_cmd_available_ = true;

    printf("Received ESP cmd: %s\n", instance_->latest_esp_cmd_);
}

void MicroRosManager::update()
{
    if (!started_ || !ros_entities_created_) {
        return;
    }

    rcl_ret_t rc = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(20));
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        ESP_LOGW(TAG, "rclc_executor_spin_some failed: %d", (int)rc);
    }
}

bool MicroRosManager::isWifiConnected()
{
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == nullptr) {
        return false;
    }

    esp_netif_ip_info_t ip_info = {};
    if (esp_netif_get_ip_info(sta_netif, &ip_info) != ESP_OK) {
        return false;
    }

    return (ip_info.ip.addr != 0);
}

bool MicroRosManager::getWifiIp(char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) {
        return false;
    }

    out[0] = '\0';

    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == nullptr) {
        return false;
    }

    esp_netif_ip_info_t ip_info = {};
    if (esp_netif_get_ip_info(sta_netif, &ip_info) != ESP_OK) {
        return false;
    }

    if (ip_info.ip.addr == 0) {
        return false;
    }

    snprintf(out, out_size, IPSTR, IP2STR(&ip_info.ip));
    return true;
}

bool MicroRosManager::hasNewPath() const
{
    return new_path_available_;
}

void MicroRosManager::consumePath(double out[][MAX_JOINTS], size_t &waypoints, size_t &dof)
{
    printf("Consuming path with %zu waypoints and %zu dof\n", latest_path_waypoints_, latest_path_dof_);
    waypoints = latest_path_waypoints_;
    dof = latest_path_dof_;

    for (size_t wp = 0; wp < waypoints; ++wp) {
        for (size_t j = 0; j < dof; ++j) {
            out[wp][j] = latest_path_[wp][j];
        }
    }

    new_path_available_ = false;
}

bool MicroRosManager::hasNewEspCmd() const
{
    return new_esp_cmd_available_;
}

void MicroRosManager::consumeEspCmd(char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) {
        return;
    }

    strncpy(out, latest_esp_cmd_, out_size - 1);
    out[out_size - 1] = '\0';
    new_esp_cmd_available_ = false;
}
