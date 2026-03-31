#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>

class RobotController;

class MicroRosController {
public:
    MicroRosController();
    void begin();
    void update(RobotController& robotController);

private:
    static void jointAnglesCallback(const void* msgin);
    static MicroRosController* _instance;

    bool connectWifiRobust(unsigned long timeout_ms = 15000);
    bool ensureWifiConnected();
    bool pingAgent(int tries = 5);
    bool createEntities();
    void destroyEntities();
    bool rebuildSession();
    void initRosObjectsToZero();

private:
    static constexpr size_t MAX_JOINTS = 10;

    const char* _ssid = "jan's Galaxy S20 FE";
    const char* _password = "12345678";
    IPAddress _agentIp = IPAddress(192, 168, 225, 96);
    size_t _agentPort = 8888;

    bool _started = false;
    bool _rosEntitiesCreated = false;
    bool _newJointCommandAvailable = false;

    unsigned long _lastDebugMs = 0;
    unsigned long _lastMsgMs = 0;
    unsigned long _lastSessionCheckMs = 0;
    unsigned long _lastRebuildAttemptMs = 0;
    unsigned long _msgCount = 0;

    static constexpr unsigned long DEBUG_PERIOD_MS = 3000;
    static constexpr unsigned long SESSION_CHECK_PERIOD_MS = 30000;
    static constexpr unsigned long REBUILD_RETRY_DELAY_MS = 1000;
    static constexpr int WIFI_MAX_ATTEMPTS = 3;

    rcl_allocator_t _allocator;
    rclc_support_t _support;
    rcl_node_t _node;
    rcl_subscription_t _subscriber;
    rclc_executor_t _executor;

    std_msgs__msg__Float64MultiArray _msg;
    double _jointBuffer[10];

    double _latestJointCommand[MAX_JOINTS] = {0.0};
    size_t _latestJointCommandSize = 0;
};