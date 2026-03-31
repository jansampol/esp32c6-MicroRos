#include "CanvasManager.h"
#include <SPI.h>
#include <WiFi.h>
#include "InputController/InputController.h"
// #include "InputController/SPI/SPI0Manager.h"   // removed for now

CanvasManager::CanvasManager(InputController& inputController)
    : _inputController(inputController)
{
}

void CanvasManager::initCanvas() {
    Serial.println("Initializing CanvasManager PSRAM buffer...");
    _canvasBuffer = (uint16_t *) ps_malloc(WIDTH * HEIGHT * sizeof(uint16_t));
    if (!_canvasBuffer) {
        Serial.println("PSRAM allocation failed!");
        return;
    }    
    _canvas = new PsGFXcanvas16(WIDTH, HEIGHT, _canvasBuffer);
}

void CanvasManager::initScreen() {
    _canvas->setFont(&FreeSans12pt7b);
    _canvas->setRotation(0);
    _canvas->fillScreen(0x0000);
    _canvas->setTextColor(ST77XX_WHITE);

    /*
    _canvas->fillScreen(0x9631);
    _canvas->setCursor(0, 50);
    _canvas->println("Hello Mamri v6!");
    */

    // Physical SPI screen transfer disabled for now
    /*
    _spi0Manager.selectDevice(SCREEN);
    _spi0Manager.getScreen().drawRGBBitmap(0, 0, _canvas->getBuffer(), WIDTH, HEIGHT);
    _spi0Manager.deselectDevice();
    */

    Serial.println("Finished screen initialization.");
}

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
    if (WiFi.status() == WL_CONNECTED) {
        display.print("IP: " + WiFi.localIP().toString());
    } else {
        display.print("WiFi: Not Connected");
    }
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
    display.print("Ferris Wheels: "); display.print(config.numOfFerrisWheels);
    display.setCursor(10, 155);
    display.print("Has Sensors: "); display.print(config.sensorsEnabled ? "Yes" : "No");
    display.setCursor(10, 180);
    display.print("Kinematics: "); display.print(robotController._kinematics ? "Yes" : "No");
    display.setCursor(10, 205);
    display.print("Control Strategy: ");
    switch (robotState.controlStrategy) {
        case PneumaticStepper::Controlstrategy::POSITION_CONTROL:
            display.print("Position");
            break;
        case PneumaticStepper::Controlstrategy::VELOCITY_CONTROL:
            display.print("Velocity");
            break;
        default:
            display.print("Unknown");
            break;
    }
    display.setCursor(10, 230);
    display.print("EE: ");
    if (robotController._kinematics) {
        switch (robotController._kinematics->getEndEffector()) {
            case EndEffector::FLANGE: display.print("Flange"); break;
            case EndEffector::NEEDLE_INSERTION: display.print("Needle Insertion"); break;
            case EndEffector::CALIBRATION: display.print("Calibration"); break;
            default: display.print("Unknown"); break;
        }
    } else {
        display.print("N/A");
    }
}

void CanvasManager::drawPageHeader(Adafruit_GFX& display, RobotName robotName) {
    display.fillRect(0, 0, display.width(), 25, rgbColor(255,200,210));
    display.setTextColor(0x0000);
    display.setTextSize(1);
    display.setCursor(5, 20);
    switch (robotName) {
        case PINK_MAMRI: display.print("PINK MAMRI"); break;
        case PURPLE_MAMRI: display.print("PURPLE MAMRI"); break;
        case STEPPER_TESTER: display.print("STEPPER TESTER"); break;
        case STEPPER_AND_FERRIS: display.print("STEPPER&FERRIS TESTER"); break;
        case NO_ROBOT: display.print("NO ROBOT"); break;
        default: display.print("UNKNOWN ROBOT"); break;
    }
}

void CanvasManager::drawStepperPositions(Adafruit_GFX& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.setTextSize(1);
    display.print("Current joint steps:");

    const auto& joints = robotState.jointSteps;
    for (size_t i = 0; i < joints.size() && i < 8; i++) {
        display.setCursor(10, 65 + i * 20);
        display.print("J");
        display.print(i + 1);
        display.print(": ");
        display.print(joints[i]);
    }
}

void CanvasManager::drawEndEffectorPosition(Adafruit_GFX& display, const RobotState& robotState) {
    display.setTextColor(0xFFFF);
    display.setCursor(10, 45);
    display.print("CURRENT POS:");

    const auto& pos = robotState.currentPosition;
    if (pos.size() >= 3) {
        display.setCursor(10, 65);
        display.print("X: "); display.print(pos[0], 1);
        display.setCursor(10, 85);
        display.print("Y: "); display.print(pos[1], 1);
        display.setCursor(10, 105);
        display.print("Z: "); display.print(pos[2], 1);
    }
    if (pos.size() == 6) {
        display.setCursor(10, 125);
        display.print("Rx: "); display.print(pos[3], 2);
        display.setCursor(10, 145);
        display.print("Ry: "); display.print(pos[4], 2);
        display.setCursor(10, 165);
        display.print("Rz: "); display.print(pos[5], 2);
    }
}

void CanvasManager::selectPage(Page page) {
    _currentPage = page;
}

uint16_t CanvasManager::rgbColor(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}