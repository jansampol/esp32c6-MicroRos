#include "InputController/InputController.h"

// ============================================================
// ESP-IDF
// ============================================================
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "pinDefinitions.h"

#include <cstdio>

static const char* TAG = "InputController";

// Constructor
InputController::InputController(InputModes defaultMode)
    : _mamriWebServer(),
      _canvasManager(*this)
{
    _defaultInputMode = defaultMode;
    _currentInputMode = defaultMode;
}

void InputController::begin() {
    // ============================================================
    // ORIGINAL ARDUINO MEMORY DEBUG
    // ============================================================
    // ESP-IDF equivalent could be added later if needed.
    //
    // Example future replacement:
    // ESP_LOGI(TAG, "Total PSRAM: %u bytes", esp_psram_get_size());
    // ESP_LOGI(TAG, "Free PSRAM: %u bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "Initializing InputController...");

    // SPI0 disabled for now
    _spi0Manager.begin();

    // Canvas initialization can already run if CanvasManager is stubbed safely
    _canvasManager.initCanvas();

    // SPI0 test disabled for now
    testSPI0Manager();

    // Screen / font/display init currently stubbed in CanvasManager
    _canvasManager.initScreen();

    // Emergency button input (active-low by default for current wiring).
    gpio_config_t emergencyConf = {};
    emergencyConf.pin_bit_mask = (1ULL << EMERGENCY_PIN);
    emergencyConf.mode = GPIO_MODE_INPUT;
    emergencyConf.pull_up_en = GPIO_PULLUP_ENABLE;
    emergencyConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    emergencyConf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&emergencyConf);

    // LED policy:
    // Green  = system on
    // Red1   = ferris wheels not ready
    // Red2   = no pressure
    // Red3   = emergency pressed
    _spi0Manager.writeGreen(true);
    _spi0Manager.writeRed1(false);
    _spi0Manager.writeRed2(false);
    _spi0Manager.writeRed3(false);

    // ============================================================
    // I2C
    // ============================================================
    #if ESP_ATTACHED
        // For now we only care about pressure sensors.
        // Ferris wheel initialization is disabled until AS5600 hardware is available.
        /*
        if(_i2cManager.begin(NUM_OF_FERRISWHEELS_ON_BOOT, 0)){
            _ferrisWheelsReady = true;
            ESP_LOGI(TAG, "Ferris wheels ready.");

            for(int i = 0; i < _i2cManager.getNumOfFerrisWheels(); i++){
                float angle = _i2cManager.readFerrisWheelAngle(i);
                (void)angle;
            }
        } else {
            ESP_LOGW(TAG, "Ferris wheels not ready.");
            _ferrisWheelsReady = false;
        }
        */

        // Still initialize I2CManager so pressure sensors can be used.
        if (_i2cManager.begin(0, 0)) {
            ESP_LOGI(TAG, "I2CManager initialized for pressure sensors.");
        } else {
            ESP_LOGW(TAG, "I2CManager failed to initialize.");
        }
    #endif

    // Ferris wheels intentionally disabled for now
    _ferrisWheelsReady = false;

    ESP_LOGI(TAG, "InputController initialized.");
}

void InputController::testSPI0Manager() {
    // SPI0 disabled for now
    ESP_LOGI(TAG, "testSPI0Manager() skipped: SPI0 disabled");
}

std::vector<float> InputController::readFerrisWheelAngles() {
    // Ferris wheels disabled for now
    return {};
}

std::vector<float> InputController::readFerrisWheelValues() {
    // Ferris wheels disabled for now
    return {};
}

void InputController::updateNumOfFerrisWheels(uint8_t numOfFerrisWheels) {
    (void)numOfFerrisWheels;

    // Ferris wheel support temporarily disabled until hardware is available
    /*
    #if ESP_ATTACHED
    ESP_LOGI(TAG, "InputController::updateNumOfFerrisWheels: calling _i2cManager.initFerrisWheels(%u)", numOfFerrisWheels);
    _ferrisWheelsReady = _i2cManager.initFerrisWheels(numOfFerrisWheels);
    #else
    _ferrisWheelsReady = false;
    #endif
    */

    _ferrisWheelsReady = false;
    ESP_LOGI(TAG, "Ferris wheel update skipped: feature disabled for now.");
}

void InputController::beginWebserver() {
    bool ok = _mamriWebServer.begin();
    if (ok) {
        ESP_LOGI(TAG, "Webserver started successfully.");
    } else {
        ESP_LOGW(TAG, "Webserver not started.");
    }
}

void InputController::webserverAttachRobotController(RobotController& robotController) {
    _mamriWebServer.attachRobotController(&robotController);
    ESP_LOGI(TAG, "RobotController attached to webserver.");
}

void InputController::update(RobotController& robotController) {
    // No SPI switches/buttons for now
    handleSwitches(robotController);

    // Blue LED disabled for now
    if (_currentInputMode != BUTTON_EE_CONTROL_MODE) {
        robotController.disableIK();
    } else {
        robotController.enableIK();
    }

    switch (_currentInputMode) {
        case BUTTON_JOINT_MODE:
            robotController.setControlStrategy(PneumaticStepper::Controlstrategy::VELOCITY_CONTROL);
            handleHorizontalButtons(robotController, true);
            break;

        case BUTTON_EE_CONTROL_MODE:
            robotController.setControlStrategy(PneumaticStepper::Controlstrategy::POSITION_CONTROL);
            handleHorizontalButtons(robotController, false);
            break;

        case JOINT_TARGET_MODE:
            robotController.setControlStrategy(PneumaticStepper::Controlstrategy::POSITION_CONTROL);
            break;

        default:
            _currentInputMode = _defaultInputMode;
            break;
    }

    _mamriWebServer.updateInputMode(_currentInputMode);
    _mamriWebServer.updateFerrisWheelsReady(_ferrisWheelsReady);

    // Display-only restore: draw UI into framebuffer and flush via SPI0 screen backend.
    if (_canvasManager.hasCanvas()) {
        _canvasManager.selectPage(mapPagePotmeter());
        _canvasManager.updateDirect(_canvasManager.getCanvas(), _currentInputMode, robotController);
        esp_err_t drawErr = _spi0Manager.drawScreenRGB565(0, 0,
                                                          CanvasManager::WIDTH,
                                                          CanvasManager::HEIGHT,
                                                          _canvasManager.getCanvas().getBuffer());
        if (drawErr != ESP_OK) {
            ESP_LOGW(TAG, "drawScreenRGB565 failed: %s", esp_err_to_name(drawErr));
        }
    } else {
        ESP_LOGW(TAG, "Canvas unavailable, skipping display update.");
    }

    // ============================================================
    // Pressure sensors enabled
    // ============================================================
    #if ESP_ATTACHED
    {
        float pressure = _i2cManager.readPressureSensor(1);

        ESP_LOGI(TAG, "Pressure sensor 1: %.4f bar", pressure);

        const bool noPressure = (pressure < 2.0f);
        _spi0Manager.writeRed2(noPressure);
    }
    #else
    _spi0Manager.writeRed2(false);
    #endif

    handleVerticalButtons(robotController);

    float maxVelocity = getMaxVelocityFromPotmeter();
    robotController.setFrequency(maxVelocity);

    // Ferris warning LED disabled for now
    _spi0Manager.writeGreen(true);

    if(!_ferrisWheelsReady && robotController.getNumOfFerrisWheels() > 0){
        _spi0Manager.writeRed1(true);
    } else {
        _spi0Manager.writeRed1(false);
    }

    // Emergency LED (red3): button pressed -> ON.
    // Current board uses active-high wiring.
    const int emergencyLevel = gpio_get_level(static_cast<gpio_num_t>(EMERGENCY_PIN));
    const bool emergencyPressed = (emergencyLevel == 1);
    _spi0Manager.writeRed3(emergencyPressed);

}

float InputController::getMaxVelocityFromPotmeter() {
    // Fixed fallback while SPI potmeter is disabled
    return 20.0f;
}

Page InputController::mapPagePotmeter()
{
    int potValue = static_cast<int>(_spi0Manager.readPotmeters(false));
    if (potValue < 171) {
        return Page::HOME;
    } else if (potValue < 400) {
        return Page::ROBOTINFO;
    } else if (potValue < 682) {
        return Page::STEPPER_POSITIONS;
    }
    return Page::END_EFFECTOR_POSITION;
}

void InputController::handleHorizontalButtons(RobotController& robotController, bool jointMode) {
    (void)robotController;
    (void)jointMode;
    // Disabled for now: depends on SPI buttons
}

void InputController::handleVerticalButtons(RobotController& robotController) {
    (void)robotController;
    // Disabled for now: depends on SPI buttons
}

void InputController::handleSwitches(RobotController& robotController) {
    (void)robotController;
    // No SPI switches for now
    // Keep current mode unchanged
}

void InputController::setMainValve(bool on) {
    _spi0Manager.setMainValve(on);
}

void InputController::setScreenRES(bool on) {
    _spi0Manager.setScreenRES(on);
}

void InputController::setScreenBLK(bool on) {
    _spi0Manager.setScreenBLK(on);
}

void InputController::setInputMode(InputModes newInputMode) {
    _currentInputMode = newInputMode;
    ESP_LOGI(TAG, "Input mode updated to %d", static_cast<int>(_currentInputMode));
}

InputModes InputController::getInputMode() {
    return _currentInputMode;
}
