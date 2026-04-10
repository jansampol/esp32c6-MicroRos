#pragma once

// ============================================================
// ORIGINAL ARDUINO / ASYNC WEB SERVER INCLUDES
// ============================================================
// Commented out for pure ESP-IDF port.
//
// #include <Arduino.h>
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <LittleFS.h>

#include "InputController/InputStructs.h"
#include "RobotController/RobotController.h"

class MamriWebServer {
public:
    MamriWebServer();

    bool begin();
    void attachRobotController(RobotController* rc);

    // Keep API for compatibility with InputController
    void updateInputMode(InputModes mode) { _currentInputMode = mode; }
    void updateFerrisWheelsReady(bool ready) { _ferrisWheelsReady = ready; }

private:
    // TEMP: async server disabled for now
    // AsyncWebServer _server;

    RobotController* _robotController = nullptr;
    InputModes _currentInputMode = BUTTON_JOINT_MODE;
    bool _ferrisWheelsReady = false;
};