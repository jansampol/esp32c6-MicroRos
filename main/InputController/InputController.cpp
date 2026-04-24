#include "InputController/InputController.h"

// ============================================================
// ESP-IDF
// ============================================================
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

    // Emergency button input (active-high by default for current wiring).
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
        if (_i2cManager.begin(NUM_OF_FERRISWHEELS_ON_BOOT, 0)) {
            _ferrisWheelsReady = true;
            ESP_LOGI(TAG, "I2CManager initialized. Ferris wheels configured: %u",
                     static_cast<unsigned>(_i2cManager.getNumOfFerrisWheels()));

            for (uint8_t i = 0; i < _i2cManager.getNumOfFerrisWheels(); i++) {
                float angle = _i2cManager.readFerrisWheelAngle(i);
                (void)angle;
            }
        } else {
            ESP_LOGW(TAG, "I2CManager init failed. Ferris wheels not ready.");
            _ferrisWheelsReady = false;
        }
    #else
        _ferrisWheelsReady = false;
    #endif

    ESP_LOGI(TAG, "InputController initialized.");
}

void InputController::testSPI0Manager() {
    // SPI0 disabled for now
    ESP_LOGI(TAG, "testSPI0Manager() skipped: SPI0 disabled");
}

std::vector<float> InputController::readFerrisWheelAngles() {
    if (!_ferrisWheelsReady) {
        return {};
    }
    return _i2cManager.readAllFerrisWheelAngles();
}

std::vector<float> InputController::readFerrisWheelValues() {
    if (!_ferrisWheelsReady) {
        return {};
    }
    return _i2cManager.readAllFerrisWheelRawValues();
}

void InputController::updateNumOfFerrisWheels(uint8_t numOfFerrisWheels) {
    #if ESP_ATTACHED
    ESP_LOGI(TAG, "InputController::updateNumOfFerrisWheels: calling _i2cManager.initFerrisWheels(%u)", numOfFerrisWheels);
    _ferrisWheelsReady = _i2cManager.initFerrisWheels(numOfFerrisWheels);
    #else
    _ferrisWheelsReady = false;
    #endif
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
    // Keep control loop fast, but limit TFT transfers to every 100 ms.
    static TickType_t lastDisplayUpdateTick = 0;
    static bool screenForcedBlack = false;
    const TickType_t now = xTaskGetTickCount();
    if ((now - lastDisplayUpdateTick) >= pdMS_TO_TICKS(100)) {
        lastDisplayUpdateTick = now;

        // At end of potmeter travel, keep display black and skip UI rendering to free time.
        const int pagePot = static_cast<int>(_spi0Manager.readPotmeters(false));
        const bool blackScreenMode = (pagePot >= 980);

        if (blackScreenMode) {
            if (!screenForcedBlack) {
                _spi0Manager.fillScreen(0x0000);
                screenForcedBlack = true;
            }
        } else {
            screenForcedBlack = false;
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
        }
    }

    // ============================================================
    // Pressure sensors enabled
    // ============================================================
    #if ESP_ATTACHED
    {
        float pressure = _i2cManager.readPressureSensor(1);

        ESP_LOGI(TAG, "Pressure sensor 1: %.4f bar", pressure);

        const bool noPressure = (pressure < 1.0f);
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
    // Test behavior:
    // emergency pressed  -> close valve
    // emergency released -> open valve
    _spi0Manager.setMainValve(!emergencyPressed);

}

float InputController::getMaxVelocityFromPotmeter() {
    const int pot = static_cast<int>(_spi0Manager.readPotmeters(true));
    const int x = (pot < 0) ? 0 : ((pot > 1023) ? 1023 : pot);

    struct Anchor {
        int pos;
        float hz;
    };

    // Calibrated to the real panel response:
    // - around 1/4 turn was reading too low (~1.7 instead of 2.0)
    // - around 3/4 turn was reading too high (~70 instead of 50)
    static const Anchor k[] = {
        {0,    1.0f},
        {179,  2.0f},
        {512, 10.0f},
        {870, 50.0f},
        {1023, 100.0f},
    };

    if (x <= k[0].pos) return k[0].hz;
    if (x >= k[4].pos) return k[4].hz;

    for (int i = 1; i < 5; ++i) {
        if (x <= k[i].pos) {
            const int x0 = k[i - 1].pos;
            const int x1 = k[i].pos;
            const float y0 = k[i - 1].hz;
            const float y1 = k[i].hz;
            const float t = static_cast<float>(x - x0) / static_cast<float>(x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    return 100.0f;
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
