#pragma once
#include <Arduino.h>
#include <vector>
#include <Adafruit_GFX.h>
#include <memory>
#include "SystemParameters.h"
#include "InputController/InputStructs.h"

#include "RobotController/RobotController.h"

// Keep only the modules that do not depend on SPI
#include "InputController/I2C/I2CManager.h"
#include "InputController/MamriWebServer.h"

// SPI-related includes removed for now
// #include "InputController/SPI/SPI0Manager.h"
// #include "InputController/CanvasManager.h"

class InputController {
    public:

        InputController(InputModes defaultMode);
        void begin();
        void beginWebserver();
        void webserverAttachRobotController(RobotController & robotController);
        void testSPI0Manager();

        // Keep API for now, but make them no-ops in the cpp
        void setMainValve(bool on);
        void setScreenRES(bool on);
        void setScreenBLK(bool on);

        void update(RobotController & robotController);

        std::vector<float> readFerrisWheelAngles();
        std::vector<float> readFerrisWheelValues();
        void updateNumOfFerrisWheels(uint8_t numOfFerrisWheels);

        //Page mapPagePotmeter();
        float getMaxVelocityFromPotmeter();

        void handleHorizontalButtons(RobotController & robotController, bool jointMode);
        void handleVerticalButtons(RobotController & robotController);
        void handleSwitches(RobotController & robotController);

        void setInputMode(InputModes newMode);
        InputModes getInputMode();

        // SPI LED call removed -> temporary no-op
        void setLedEmergency(bool on){
            (void)on;
        }

        bool getFerrisWheelsReady(){
            return _ferrisWheelsReady;
        }

        // Remove this for now because it exposes SPI directly
        // SPI0Manager& getSPI0Manager(){ return _spi0Manager; }

    private:
        // SPI0Manager _spi0Manager;   // removed for now
        I2CManager _i2cManager;

        InputModes _defaultInputMode;
        InputModes _currentInputMode;

        MamriWebServer _mamriWebServer;

        // Canvas depends on SPI0Manager, so remove for now
        // CanvasManager _canvasManager;

        bool _ferrisWheelsReady = false;

        uint16_t _lastVerticalButtonState = 0;

    public:
        ~InputController() = default;
};