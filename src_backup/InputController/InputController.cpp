#include "InputController/InputController.h"

// Constructor
InputController::InputController(InputModes defaultMode)
    :
    _i2cManager(),
    _mamriWebServer()
    // _canvasManager(*this, _spi0Manager)
{
    _defaultInputMode = defaultMode;
    _currentInputMode = defaultMode;
}

void InputController::begin() {
    Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

    // SPI0 disabled for now
    // _spi0Manager.begin();

    // Canvas disabled for now
    // _canvasManager.initCanvas();

    // SPI0 test disabled for now
    // testSPI0Manager();

    // Screen disabled for now
    // _canvasManager.initScreen();

    #if ESP_ATTACHED
        if(_i2cManager.begin(NUM_OF_FERRISWHEELS_ON_BOOT, 0)){
            _ferrisWheelsReady = true;
            Serial.println();
            for(int i = 0; i < _i2cManager.getNumOfFerrisWheels(); i++){
                float angle = _i2cManager.readFerrisWheelAngle(i);
                (void)angle;
            }
            Serial.println("Ferris wheels ready.");
        } else {
            Serial.println("Ferris wheels not ready.");
            _ferrisWheelsReady = false;
        }
    #endif

    // Potmeter reads disabled for now
    // Serial.println("Re-read potmeters:");
    // Serial.print("ADC Potmeter 0: ");
    // Serial.println(_spi0Manager.readPotmeters(0));
    // Serial.print("ADC Potmeter 1: ");
    // Serial.println(_spi0Manager.readPotmeters(1));
}

void InputController::testSPI0Manager() {
    // SPI0 disabled for now
    Serial.println("testSPI0Manager() skipped: SPI0 disabled");
}

std::vector<float> InputController::readFerrisWheelAngles(){
    if(!_ferrisWheelsReady){
        return {};
    }
    return _i2cManager.readAllFerrisWheelAngles();
}

std::vector<float> InputController::readFerrisWheelValues(){
    if(!_ferrisWheelsReady){
        return {};
    }
    return _i2cManager.readAllFerrisWheelRawValues();
}

void InputController::updateNumOfFerrisWheels(uint8_t numOfFerrisWheels){
    #if ESP_ATTACHED
    Serial.printf("InputController::updateNumOfFerrisWheels: calling _i2cManager.initFerrisWheels(%d)\n", numOfFerrisWheels);
    _ferrisWheelsReady = _i2cManager.initFerrisWheels(numOfFerrisWheels);
    #else
    _ferrisWheelsReady = false;
    #endif
}

void InputController::beginWebserver() {
    _mamriWebServer.begin();
}

void InputController::webserverAttachRobotController(RobotController & robotController) {
    _mamriWebServer.attachRobotController(&robotController);
}

void InputController::update(RobotController & robotController) {
    // No SPI switches/buttons for now
    handleSwitches(robotController);

    // Blue LED disabled for now
    if(_currentInputMode != BUTTON_EE_CONTROL_MODE){
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

    // Canvas and screen output disabled for now
    // _canvasManager.selectPage(mapPagePotmeter());
    // _canvasManager.updateDirect(_canvasManager.getCanvas(), _currentInputMode, robotController);
    // _spi0Manager.selectDevice(SCREEN);
    // uint16_t* buf = (uint16_t*)_canvasManager.getCanvas().getBuffer();
    // _spi0Manager.getScreen().drawRGBBitmap(0, 0, buf, _canvasManager.WIDTH, _canvasManager.HEIGHT);
    // _spi0Manager.deselectDevice();

    // Pressure LED disabled for now
    // #if ESP_ATTACHED
    // if(_i2cManager.readPressureSensor(0) < 2.0f){
    //     _spi0Manager.writeRed2(true);
    // } else {
    //     _spi0Manager.writeRed2(false);
    // }
    // #endif

    handleVerticalButtons(robotController);

    float maxVelocity = getMaxVelocityFromPotmeter();
    robotController.setFrequency(maxVelocity);

    // Ferris warning LED disabled for now
    // if(!_ferrisWheelsReady && robotController.getNumOfFerrisWheels() > 0){
    //     _spi0Manager.writeRed1(true);
    // } else {
    //     _spi0Manager.writeRed1(false);
    // }
}
/*
Page InputController::mapPagePotmeter() {
    // No SPI potmeter for now
    return Page::HOME;
}
*/
float InputController::getMaxVelocityFromPotmeter()
{
    // Fixed fallback while SPI potmeter is disabled
    return 20.0f;
}

void InputController::handleHorizontalButtons(RobotController & robotController, bool jointMode){
    (void)robotController;
    (void)jointMode;
    // Disabled for now: depends on SPI buttons
}

void InputController::handleVerticalButtons(RobotController & robotController){
    (void)robotController;
    // Disabled for now: depends on SPI buttons
}

void InputController::handleSwitches(RobotController & robotController){
    (void)robotController;
    // No SPI switches for now
    // Keep current mode unchanged
}

void InputController::setMainValve(bool on) {
    (void)on;
    // Disabled for now: depends on SPI0
}

void InputController::setScreenRES(bool on) {
    (void)on;
    // Disabled for now: depends on SPI0
}

void InputController::setScreenBLK(bool on) {
    (void)on;
    // Disabled for now: depends on SPI0
}

void InputController::setInputMode(InputModes newInputMode) {
    _currentInputMode = newInputMode;
}

InputModes InputController::getInputMode() {
    return _currentInputMode;
}