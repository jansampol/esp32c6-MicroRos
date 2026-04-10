#define ENABLE_ADAFRUIT_DISPLAY 0
#include "CanvasManager.h"

#include <cstdio>

// =============================
// ESP-IDF
// =============================
#include "esp_log.h"
#include "esp_heap_caps.h"

// =============================
// FUTURE / NOT CURRENTLY USED
// =============================
#include "InputController/InputController.h"

static const char* TAG = "CanvasManager";

// ============================================================
// Toggle display backend
// ============================================================
#define ENABLE_ADAFRUIT_DISPLAY 0

CanvasManager::CanvasManager(InputController& inputController)
    : _inputController(inputController)
{
}

void CanvasManager::initCanvas() {
    ESP_LOGI(TAG, "Initializing CanvasManager...");

#if ENABLE_ADAFRUIT_DISPLAY
    // =========================================================================
    // ORIGINAL CODE (Adafruit / Arduino)
    // =========================================================================
    _canvasBuffer = static_cast<uint16_t*>(
        heap_caps_malloc(WIDTH * HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM)
    );

    if (!_canvasBuffer) {
        ESP_LOGE(TAG, "PSRAM allocation failed!");
        return;
    }

    _canvas = new PsGFXcanvas16(WIDTH, HEIGHT, _canvasBuffer);

    if (!_canvas) {
        ESP_LOGE(TAG, "Canvas allocation failed!");
        return;
    }

    ESP_LOGI(TAG, "Canvas buffer initialized successfully.");
#else
    _canvasBuffer = nullptr;
    ESP_LOGI(TAG, "Canvas disabled (no display backend).");
#endif
}

void CanvasManager::initScreen() {

#if ENABLE_ADAFRUIT_DISPLAY
    if (!_canvas) {
        ESP_LOGW(TAG, "initScreen() called before initCanvas()");
        return;
    }

    _canvas->setFont(&FreeSans12pt7b);
    _canvas->setRotation(0);
    _canvas->fillScreen(0x0000);
    _canvas->setTextColor(ST77XX_WHITE);

    ESP_LOGI(TAG, "Finished screen initialization.");
#else
    ESP_LOGI(TAG, "initScreen skipped (display disabled).");
#endif
}

#if ENABLE_ADAFRUIT_DISPLAY

void CanvasManager::updateDirect(Adafruit_GFX& display, InputModes mode, const RobotController& robotController) {
    display.fillScreen(0x0000);
    drawPageHeader(display, robotController.getRobotConfig().name);

    switch (_currentPage) {
        case HOME:
            drawHome(display, mode, robotController.getRobotState());
            break;

        case ROBOTINFO:
            drawRobotInfoScreen(display, robotController.getRobotState(), robotController);
            break;

        case STEPPER_POSITIONS:
            drawStepperPositions(display, robotController.getRobotState());
            break;

        case END_EFFECTOR_POSITION:
            drawEndEffectorPosition(display, robotController.getRobotState());
            break;

        default:
            break;
    }
}

void CanvasManager::drawHome(Adafruit_GFX& display, InputModes mode, const RobotState& robotState) {
    (void)robotState;

    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.print("MAMRI CONTROLLER");

    display.setCursor(10, 80);
    display.print("Mode: ");

    switch (mode) {
        case BUTTON_JOINT_MODE: display.print("JOINT VELOCITY"); break;
        case BUTTON_EE_CONTROL_MODE: display.print("EE CONTROL"); break;
        case JOINT_TARGET_MODE: display.print("JOINT TARGET"); break;
        case WEBPAGE_MODE: display.print("WEBPAGE"); break;
        case TOUCHSCREEN_MODE: display.print("TOUCHSCREEN"); break;
        case HANDTOOL_MODE: display.print("HANDTOOL"); break;
        case SPACEMOUSE_MODE: display.print("SPACEMOUSE"); break;
        default: display.print("UNKNOWN"); break;
    }

    display.setCursor(10, 120);
    display.print("Use the knob to change");
    display.setCursor(10, 145);
    display.print("page.");

    display.setCursor(10, 185);
    display.print("WiFi: N/A");
}

void CanvasManager::drawRobotInfoScreen(Adafruit_GFX& display, const RobotState& robotState, const RobotController& robotController) {
    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.print("ROBOT INFO");

    const auto& config = robotController.getRobotConfig();

    display.setCursor(10, 80);
    display.print("Motor freq: ");
    display.print(_inputController.getMaxVelocityFromPotmeter());

    display.setCursor(10, 105);
    display.printf("DOF/steppers: %d/%d", config.degreesOfFreedom, config.numOfSteppers);

    display.setCursor(10, 130);
    display.print("Ferris Wheels: ");
    display.print(config.numOfFerrisWheels);

    display.setCursor(10, 155);
    display.print("Has Sensors: ");
    display.print(config.sensorsEnabled ? "Yes" : "No");
}

void CanvasManager::drawPageHeader(Adafruit_GFX& display, RobotName robotName) {
    display.fillRect(0, 0, display.width(), 25, rgbColor(255, 200, 210));
    display.setTextColor(0x0000);
    display.setCursor(5, 20);

    switch (robotName) {
        case PINK_MAMRI: display.print("PINK MAMRI"); break;
        case PURPLE_MAMRI: display.print("PURPLE MAMRI"); break;
        default: display.print("UNKNOWN"); break;
    }
}

void CanvasManager::drawStepperPositions(Adafruit_GFX& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.print("Current joint steps:");

    const auto& joints = robotState.jointSteps;

    for (size_t i = 0; i < joints.size() && i < 8; i++) {
        display.setCursor(10, 65 + static_cast<int>(i) * 20);
        display.print("J");
        display.print(static_cast<int>(i + 1));
        display.print(": ");
        display.print(joints[i]);
    }
}

void CanvasManager::drawEndEffectorPosition(Adafruit_GFX& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.print("CURRENT POS:");
}

#endif // ENABLE_ADAFRUIT_DISPLAY

void CanvasManager::selectPage(Page page) {
    _currentPage = page;
}

uint16_t CanvasManager::rgbColor(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xF8) << 8) |
                                 ((g & 0xFC) << 3) |
                                 (b >> 3));
}