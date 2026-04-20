#pragma once

// ============================================================
// ORIGINAL ARDUINO INCLUDES
// ============================================================
// Commented out for ESP-IDF porting.

// #include <Adafruit_GFX.h>

#include <vector>
#include <memory>
#include "../SystemParameters.h"
#include "InputController/InputStructs.h"

#include "RobotController/RobotController.h"

// ============================================================
// I2C / WEB / DISPLAY MODULES
// ============================================================

// TEMP: I2C manager disabled for now until ESP-IDF I2C port is ready
#include "InputController/I2C/I2CManager.h"

// Web server can stay only if already ported and compiling
#include "InputController/MamriWebServer.h"

// SPI-related includes removed for now
#include "InputController/SPI/SPI0Manager.h"
#include "InputController/CanvasManager.h"

class InputController {
public:
    InputController(InputModes defaultMode);

    void begin();
    void beginWebserver();
    void webserverAttachRobotController(RobotController& robotController);
    void testSPI0Manager();

    // Keep API for now, but make them no-ops in the cpp
    void setMainValve(bool on);
    void setScreenRES(bool on);
    void setScreenBLK(bool on);

    void update(RobotController& robotController);

    std::vector<float> readFerrisWheelAngles();
    std::vector<float> readFerrisWheelValues();
    void updateNumOfFerrisWheels(uint8_t numOfFerrisWheels);

    Page mapPagePotmeter();
    float getMaxVelocityFromPotmeter();

    void handleHorizontalButtons(RobotController& robotController, bool jointMode);
    void handleVerticalButtons(RobotController& robotController);
    void handleSwitches(RobotController& robotController);

    void setInputMode(InputModes newMode);
    InputModes getInputMode();

    void setLedEmergency(bool on) {
        _spi0Manager.writeRed3(on);
    }

    bool getFerrisWheelsReady() {
        return _ferrisWheelsReady;
    }

    // Remove this for now because it exposes SPI directly
    // SPI0Manager& getSPI0Manager(){ return _spi0Manager; }

private:
    // ============================================================
    // TEMP: SPI / I2C / Canvas backends disabled
    // ============================================================

    SPI0Manager _spi0Manager;   // removed for now

    // TEMP: I2C manager disabled for now
    I2CManager _i2cManager;

    InputModes _defaultInputMode;
    InputModes _currentInputMode;

    MamriWebServer _mamriWebServer;

    // Canvas depends on SPI0Manager and font/display stack, so remove for now
    CanvasManager _canvasManager;

    bool _ferrisWheelsReady = false;

    uint16_t _lastVerticalButtonState = 0;

public:
    ~InputController() = default;
};
