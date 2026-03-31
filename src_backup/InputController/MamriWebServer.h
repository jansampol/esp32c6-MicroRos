#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "SystemParameters.h"
#include "InputController/InputStructs.h"
#include "RobotController/RobotController.h"
#include "RobotController/RobotStructs.h"

class MamriWebServer {
public:
    MamriWebServer();
    bool begin();
    void attachRobotController(RobotController* rc);
    void updateInputMode(InputModes mode) { _currentInputMode = mode; }
    void updateFerrisWheelsReady(bool ready) { _ferrisWheelsReady = ready; }


private:
    AsyncWebServer _server{80};
    RobotController* _robotController = nullptr;
    InputModes _currentInputMode = InputModes::BUTTON_JOINT_MODE;
    bool _ferrisWheelsReady = false;
};
