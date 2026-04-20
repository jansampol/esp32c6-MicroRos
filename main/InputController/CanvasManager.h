#pragma once

// =============================
// ESP-IDF / C++ replacements
// =============================
#include <cstdint>

#include "InputController/DisplayCanvas.h"
#include "InputController/InputStructs.h"
#include "../RobotStructs.h"
#include "RobotController/RobotController.h"

enum Page {
    HOME,
    ROBOTINFO,
    STEPPER_POSITIONS,
    END_EFFECTOR_POSITION,
};

class InputController;

class CanvasManager {
public:
    constexpr static int WIDTH = 320;
    constexpr static int HEIGHT = 240;

    CanvasManager(InputController& inputController);

    void initCanvas();
    void initScreen();

    void updateDirect(DisplayCanvas& display, InputModes mode, const RobotController& robotController);

    void selectPage(Page page);

    uint16_t rgbColor(uint8_t r, uint8_t g, uint8_t b);

    DisplayCanvas& getCanvas() { return *_canvas; }
    bool hasCanvas() const { return _canvas != nullptr; }

private:
    void drawHome(DisplayCanvas& display, InputModes mode, const RobotState& robotState);
    void drawRobotInfoScreen(DisplayCanvas& display, const RobotState& robotState, const RobotController& robotController);
    void drawStepperPositions(DisplayCanvas& display, const RobotState& robotState);
    void drawEndEffectorPosition(DisplayCanvas& display, const RobotState& robotState);
    void drawPageHeader(DisplayCanvas& display, RobotName robotName);

private:
    Page _currentPage = Page::HOME;
    int _pageIndex = 0;  // currently unused, kept for future parity with original design

    uint16_t* _canvasBuffer = nullptr;
    DisplayCanvas* _canvas = nullptr;

    InputController& _inputController;
};
