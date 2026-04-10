#pragma once

// =============================
// ESP-IDF / C++ replacements
// =============================
#include <cstdint>

// =============================
// TEMP: Adafruit display stack disabled
// =============================
// These were used in the original PlatformIO/Arduino version:
//
// #include <Adafruit_GFX.h>
// #include <Adafruit_ST7789.h>
// #include <Fonts/FreeSans12pt7b.h>
//
// For the pure ESP-IDF port, they are disabled for now.
// When display support is re-enabled, restore these includes
// or replace them with a native ESP-IDF display stack.

// =============================
// PROJECT HEADERS
// =============================
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

// ============================================================================
// TEMP: Adafruit canvas removed during ESP-IDF port
// ----------------------------------------------------------------------------
// Original code used:
//   class PsGFXcanvas16 : public GFXcanvas16 { ... };
// This depends on Adafruit_GFX, so it is disabled for now.
// When display support is restored, add it back.
// ============================================================================
//
// class PsGFXcanvas16 : public GFXcanvas16 {
// public:
//     PsGFXcanvas16(int16_t w, int16_t h, uint16_t* external_buffer)
//         : GFXcanvas16(w, h, false)
//     {
//         buffer = external_buffer;
//     }
// };

class CanvasManager {
public:
    constexpr static int WIDTH = 320;
    constexpr static int HEIGHT = 240;

    CanvasManager(InputController& inputController);

    void initCanvas();
    void initScreen();

    // TEMP: display update disabled because Adafruit_GFX is removed
    // void updateDirect(Adafruit_GFX& display, InputModes mode, const RobotController& robotController);

    void selectPage(Page page);

    uint16_t rgbColor(uint8_t r, uint8_t g, uint8_t b);

    // TEMP: canvas accessor disabled because PsGFXcanvas16 is removed
    // PsGFXcanvas16& getCanvas() { return *_canvas; }

private:
    // TEMP: all drawing functions disabled because they depend on Adafruit_GFX
    // void drawHome(Adafruit_GFX& display, InputModes mode, const RobotState& robotState);
    // void drawRobotInfoScreen(Adafruit_GFX& display, const RobotState& robotState, const RobotController& robotController);
    // void drawStepperPositions(Adafruit_GFX& display, const RobotState& robotState);
    // void drawEndEffectorPosition(Adafruit_GFX& display, const RobotState& robotState);
    // void drawPageHeader(Adafruit_GFX& display, RobotName robotName);

private:
    Page _currentPage = Page::HOME;
    int _pageIndex = 0;  // currently unused, kept for future parity with original design

    // TEMP: Adafruit canvas storage removed for now
    uint16_t* _canvasBuffer = nullptr;
    // PsGFXcanvas16* _canvas = nullptr;

    InputController& _inputController;

    // =========================================================================
    // ORIGINAL / FUTURE
    // =========================================================================
    // SPI0Manager was removed for now during porting.
    // When display transfer is re-enabled, add back the screen/SPI dependency.
    //
    // SPI0Manager& _spi0Manager;
};