#include "CanvasManager.h"

#include <cstdio>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "InputController/InputController.h"
#include "MicroRosController/MicroRosManager.h"

static const char* TAG = "CanvasManager";

CanvasManager::CanvasManager(InputController& inputController)
    : _inputController(inputController)
{
}

void CanvasManager::initCanvas() {
    ESP_LOGI(TAG, "Initializing CanvasManager...");

    _canvasBuffer = static_cast<uint16_t*>(
        heap_caps_malloc(WIDTH * HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM)
    );
    if (!_canvasBuffer) {
        ESP_LOGW(TAG, "PSRAM allocation failed, falling back to internal RAM.");
        _canvasBuffer = static_cast<uint16_t*>(
            heap_caps_malloc(WIDTH * HEIGHT * sizeof(uint16_t), MALLOC_CAP_8BIT)
        );
    }

    if (!_canvasBuffer) {
        ESP_LOGE(TAG, "Canvas buffer allocation failed!");
        return;
    }

    _canvas = new DisplayCanvas(WIDTH, HEIGHT, _canvasBuffer);

    if (!_canvas) {
        ESP_LOGE(TAG, "Canvas allocation failed!");
        return;
    }

    _canvas->fillScreen(0x0000);
    _canvas->setTextColor(0xFFFF);
    _canvas->setTextSize(2);
    ESP_LOGI(TAG, "Canvas buffer initialized successfully.");
}

void CanvasManager::initScreen() {
    if (!_canvas) {
        ESP_LOGW(TAG, "initScreen() called before initCanvas()");
        return;
    }

    _canvas->fillScreen(0x0000);
    _canvas->setTextColor(0xFFFF);
    _canvas->setTextSize(2);
    ESP_LOGI(TAG, "Finished screen initialization.");
}

void CanvasManager::updateDirect(DisplayCanvas& display, InputModes mode, const RobotController& robotController) {
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

void CanvasManager::drawHome(DisplayCanvas& display, InputModes mode, const RobotState& robotState) {
    (void)robotState;

    display.setTextColor(0xFFFF);
    display.setTextSize(2);
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
    char ip_str[20] = {0};
    if (MicroRosManager::getWifiIp(ip_str, sizeof(ip_str))) {
        display.print("IP: ");
        display.print(ip_str);
    } else if (MicroRosManager::isWifiConnected()) {
        display.print("WiFi: Connected");
    } else {
        display.print("WiFi: Not Connected");
    }
}

void CanvasManager::drawRobotInfoScreen(DisplayCanvas& display, const RobotState& robotState, const RobotController& robotController) {
    (void)robotState;
    display.setTextColor(0xFFFF);
    display.setTextSize(2);
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
    display.print(static_cast<unsigned>(config.numOfFerrisWheels));

    display.setCursor(10, 155);
    display.print("Has Sensors: ");
    display.print(config.sensorsEnabled ? "Yes" : "No");
}

void CanvasManager::drawPageHeader(DisplayCanvas& display, RobotName robotName) {
    display.fillRect(0, 0, display.width(), 32, rgbColor(255, 200, 210));
    display.setTextColor(0x0000);
    display.setTextSize(2);
    display.setCursor(5, 8);

    switch (robotName) {
        case PINK_MAMRI: display.print("PINK MAMRI"); break;
        case PURPLE_MAMRI: display.print("PURPLE MAMRI"); break;
        case STEPPER_TESTER: display.print("STEPPER TESTER"); break;
        case STEPPER_AND_FERRIS: display.print("STEPPER&FERRIS"); break;
        case NO_ROBOT: display.print("NO ROBOT"); break;
        default: display.print("UNKNOWN"); break;
    }
}

void CanvasManager::drawStepperPositions(DisplayCanvas& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setTextSize(2);
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

void CanvasManager::drawEndEffectorPosition(DisplayCanvas& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setTextSize(2);
    display.setCursor(10, 45);
    display.print("CURRENT POS:");

    const auto& pos = robotState.currentPosition;
    if (pos.size() >= 3) {
        display.setCursor(10, 70);  display.print("X: ");  display.print(pos[0], 1);
        display.setCursor(10, 95);  display.print("Y: ");  display.print(pos[1], 1);
        display.setCursor(10, 120); display.print("Z: ");  display.print(pos[2], 1);
    }
    if (pos.size() >= 6) {
        display.setCursor(10, 145); display.print("RX: "); display.print(pos[3], 2);
        display.setCursor(10, 170); display.print("RY: "); display.print(pos[4], 2);
        display.setCursor(10, 195); display.print("RZ: "); display.print(pos[5], 2);
    }
}

void CanvasManager::selectPage(Page page) {
    _currentPage = page;
}

uint16_t CanvasManager::rgbColor(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xF8) << 8) |
                                 ((g & 0xFC) << 3) |
                                 (b >> 3));
}
