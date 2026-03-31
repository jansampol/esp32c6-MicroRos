/*
    This class manages the state of the menu and canvas. 
    This is the GFX object, which is later rendered by the SPI0Manager's TFT screen.
    The canvas is owned by the InputController, which passes it to the CanvasManager to draw on it.
    Splitting the drawing logic keeps the InputController cleaner and prevents drawing code from being mixed with input handling code.
*/

#pragma once
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans12pt7b.h> // 18 high; one line is 29 pixels
#include "InputController/InputStructs.h"
#include "RobotController/RobotStructs.h"
#include "RobotController/RobotController.h"

enum Page { 
    HOME,
    ROBOTINFO,
    STEPPER_POSITIONS,
    END_EFFECTOR_POSITION,
};

class InputController;
// class SPI0Manager;   // removed for now

/* we make our own subclass of GFXcanvas16 so that we can use our own buffer */
class PsGFXcanvas16 : public GFXcanvas16 {
public:
  PsGFXcanvas16(int16_t w, int16_t h, uint16_t* external_buffer)
      : GFXcanvas16(w, h, false) {
        buffer = external_buffer;
      }
};

class CanvasManager {
public:
    constexpr static int WIDTH = 320;
    constexpr static int HEIGHT = 240;
    constexpr static int TFT_DC = 9;

    // SPI0Manager removed from constructor
    CanvasManager(InputController& inputController);

    void initCanvas();
    void initScreen();
    
    void updateDirect(Adafruit_GFX& display, InputModes mode, const RobotController& robotController);
    
    void selectPage(Page page);

    uint16_t rgbColor(uint8_t r, uint8_t g, uint8_t b);

    PsGFXcanvas16& getCanvas() { return *_canvas; }

private:
    void drawHome(Adafruit_GFX& display, InputModes mode, const RobotState& robotState);
    void drawRobotInfoScreen(Adafruit_GFX& display, const RobotState& robotState, const RobotController& robotController);
    void drawStepperPositions(Adafruit_GFX& display, const RobotState& robotState);
    void drawEndEffectorPosition(Adafruit_GFX& display, const RobotState& robotState);
    
    void drawPageHeader(Adafruit_GFX& display, RobotName robotName);

    Page _currentPage = Page::HOME;
    int _pageIndex = 0;

    uint16_t *_canvasBuffer = nullptr;
    PsGFXcanvas16 *_canvas = nullptr;

    InputController& _inputController;
    // SPI0Manager& _spi0Manager;   // removed for now
};