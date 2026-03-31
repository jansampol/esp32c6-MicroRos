#include "MicroRosController/MicroRosController.h"
#include "RobotController/RobotController.h"
#include <vector>
MicroRosController* MicroRosController::_instance = nullptr;

MicroRosController::MicroRosController() {
    initRosObjectsToZero();
}

void MicroRosController::initRosObjectsToZero() {
    _node = rcl_get_zero_initialized_node();
    _subscriber = rcl_get_zero_initialized_subscription();
    _executor = rclc_executor_get_zero_initialized_executor();
}

bool MicroRosController::connectWifiRobust(unsigned long timeout_ms) {
    Serial.println("WiFi: reset state");

    WiFi.disconnect(true, true);
    delay(1000);

    WiFi.mode(WIFI_OFF);
    delay(1000);

    WiFi.mode(WIFI_STA);
    delay(1000);

    WiFi.setSleep(false);

    Serial.println("WiFi: begin");
    WiFi.begin(_ssid, _password);

    unsigned long start = millis();

    while (millis() - start < timeout_ms) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi connected");
            Serial.print("ESP32 IP: ");
            Serial.println(WiFi.localIP());
            return true;
        }
        Serial.print(".");
        delay(250);
    }

    Serial.println();
    Serial.println("WiFi connect timeout");
    return false;
}

bool MicroRosController::ensureWifiConnected() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.println("WiFi not connected. Reconnecting...");

    for (int attempt = 1; attempt <= WIFI_MAX_ATTEMPTS; attempt++) {
        Serial.print("WiFi reconnect attempt ");
        Serial.println(attempt);

        if (connectWifiRobust()) {
            return true;
        }

        delay(1000);
    }

    return false;
}

bool MicroRosController::pingAgent(int tries) {
    Serial.println("Pinging agent...");

    for (int i = 0; i < tries; i++) {
        rmw_ret_t rc = rmw_uros_ping_agent(1000, 1);

        Serial.print("Ping ");
        Serial.print(i + 1);
        Serial.print(": rc = ");
        Serial.println((int)rc);

        if (rc == RMW_RET_OK) {
            return true;
        }

        delay(500);
    }

    return false;
}

void MicroRosController::jointAnglesCallback(const void* msgin) {
    if (_instance == nullptr) return;

    const auto* in = (const std_msgs__msg__Float64MultiArray*)msgin;

    size_t n = in->data.size;
    if (n > MAX_JOINTS) n = MAX_JOINTS;

    for (size_t i = 0; i < n; i++) {
        _instance->_latestJointCommand[i] = in->data.data[i];
    }

    _instance->_latestJointCommandSize = n;
    _instance->_newJointCommandAvailable = true;
    _instance->_msgCount++;
    _instance->_lastMsgMs = millis();

    Serial.print("Received joint angles: ");
    for (size_t i = 0; i < n; i++) {
        Serial.print(_instance->_latestJointCommand[i], 6);
        Serial.print(" ");
    }
    Serial.println();
}

bool MicroRosController::createEntities() {
    _allocator = rcl_get_default_allocator();
    initRosObjectsToZero();

    rcl_ret_t rc = rclc_support_init(&_support, 0, NULL, &_allocator);
    Serial.print("rclc_support_init rc = ");
    Serial.println((int)rc);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_node_init_default(&_node, "esp32_joint_subscriber", "", &_support);
    Serial.print("rclc_node_init_default rc = ");
    Serial.println((int)rc);
    if (rc != RCL_RET_OK) {
        rclc_support_fini(&_support);
        return false;
    }

    _msg.data.data = _jointBuffer;
    _msg.data.capacity = 10;
    _msg.data.size = 0;

    rc = rclc_subscription_init_default(
        &_subscriber,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/mamri/planning/joint_angles"
    );
    Serial.print("rclc_subscription_init_default rc = ");
    Serial.println((int)rc);
    if (rc != RCL_RET_OK) {
        rcl_node_fini(&_node);
        rclc_support_fini(&_support);
        return false;
    }

    rc = rclc_executor_init(&_executor, &_support.context, 1, &_allocator);
    Serial.print("rclc_executor_init rc = ");
    Serial.println((int)rc);
    if (rc != RCL_RET_OK) {
        rcl_subscription_fini(&_subscriber, &_node);
        rcl_node_fini(&_node);
        rclc_support_fini(&_support);
        return false;
    }

    rc = rclc_executor_add_subscription(
        &_executor,
        &_subscriber,
        &_msg,
        &MicroRosController::jointAnglesCallback,
        ON_NEW_DATA
    );
    Serial.print("rclc_executor_add_subscription rc = ");
    Serial.println((int)rc);
    if (rc != RCL_RET_OK) {
        rclc_executor_fini(&_executor);
        rcl_subscription_fini(&_subscriber, &_node);
        rcl_node_fini(&_node);
        rclc_support_fini(&_support);
        return false;
    }

    _rosEntitiesCreated = true;
    _lastMsgMs = millis();
    _lastSessionCheckMs = millis();

    Serial.println("Subscriber ready.");
    return true;
}

void MicroRosController::destroyEntities() {
    if (!_rosEntitiesCreated) {
        return;
    }

    Serial.println("Destroying micro-ROS entities...");

    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    if (rmw_context != NULL) {
        rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    rclc_executor_fini(&_executor);
    rcl_subscription_fini(&_subscriber, &_node);
    rcl_node_fini(&_node);
    rclc_support_fini(&_support);

    _rosEntitiesCreated = false;
    initRosObjectsToZero();
}

bool MicroRosController::rebuildSession() {
    Serial.println("=== FULL micro-ROS SESSION REBUILD ===");

    destroyEntities();

    if (!ensureWifiConnected()) {
        Serial.println("Rebuild failed: WiFi not connected");
        return false;
    }

    set_microros_wifi_transports(
        const_cast<char*>(_ssid),
        const_cast<char*>(_password),
        _agentIp,
        _agentPort
    );

    delay(1000);

    if (!pingAgent()) {
        Serial.println("Rebuild failed: agent not reachable");
        return false;
    }

    if (!createEntities()) {
        Serial.println("Rebuild failed: entity creation failed");
        return false;
    }

    Serial.println("=== REBUILD SUCCESS ===");
    return true;
}

void MicroRosController::begin() {
    _instance = this;

    Serial.println("BOOT: WiFi + subscriber + recovery");

    bool wifi_ok = false;
    for (int attempt = 1; attempt <= WIFI_MAX_ATTEMPTS && !wifi_ok; attempt++) {
        Serial.print("BOOT: WiFi attempt ");
        Serial.println(attempt);

        wifi_ok = connectWifiRobust();

        if (!wifi_ok) {
            Serial.println("BOOT: retrying...");
            delay(2000);
        }
    }

    if (!wifi_ok) {
        Serial.println("BOOT: WiFi failed after retries");
        return;
    }

    if (!rebuildSession()) {
        Serial.println("BOOT: initial rebuild failed, loop() will retry");
        return;
    }

    _started = true;
    Serial.println("BOOT: setup finished");
}

void MicroRosController::update(RobotController& robotController) {
    unsigned long now = millis();

    if (WiFi.status() != WL_CONNECTED) {
        if (now - _lastRebuildAttemptMs > REBUILD_RETRY_DELAY_MS) {
            _lastRebuildAttemptMs = now;
            Serial.println("WiFi lost. Rebuilding session...");
            rebuildSession();
        }
        return;
    }

    if (!_rosEntitiesCreated) {
        if (now - _lastRebuildAttemptMs > REBUILD_RETRY_DELAY_MS) {
            _lastRebuildAttemptMs = now;
            Serial.println("No active micro-ROS session. Retrying rebuild...");
            rebuildSession();
        }
        return;
    }

    rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(20));

    if (_newJointCommandAvailable) {
        std::vector<float> jointAngles;
        jointAngles.reserve(_latestJointCommandSize);

        for (size_t i = 0; i < _latestJointCommandSize; i++) {
            jointAngles.push_back((float)_latestJointCommand[i]);
        }

        robotController.setJointTargetRad(jointAngles);
        _newJointCommandAvailable = false;

        Serial.println("Applied ROS joint target to RobotController");
    }

    if (now - _lastSessionCheckMs > SESSION_CHECK_PERIOD_MS) {
        _lastSessionCheckMs = now;

        Serial.println("Periodic session health check...");
        if (!pingAgent(2)) {
            Serial.println("Session health check failed. Rebuilding...");
            rebuildSession();
        } else {
            Serial.println("Session health check OK");
        }
    }

    if (now - _lastDebugMs > DEBUG_PERIOD_MS) {
        _lastDebugMs = now;

        Serial.print("[DEBUG] IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(" | RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.print(" | Msg count: ");
        Serial.print(_msgCount);
        Serial.print(" | Time since last msg [ms]: ");
        Serial.println(now - _lastMsgMs);
    }
}