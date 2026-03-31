#include "RobotController/RobotController.h"

#include <algorithm>

RobotController::RobotController() :
    _kinematics()
{
}

void RobotController::begin() {
    _spi1Manager.begin();
    turnOffValves();

    setupPurpleMamri();
    resetSteppers();
    resetPositions();

    Serial.println("- Switch to PURPLE_MAMRI");
}

// -------------------------------------------------------------------------------------------------
// Minimal robot selection API
// -------------------------------------------------------------------------------------------------

void RobotController::nextRobot() {
    // Minimal version: always stay on Purple
    changeRobot(PURPLE_MAMRI);
}

void RobotController::changeRobot(RobotName robotName, int numOfSteppersAndFerris) {
    (void)robotName;
    (void)numOfSteppersAndFerris;

    // Minimal version: always configure Purple only
    setupPurpleMamri();
    Serial.println("- Switch to PURPLE_MAMRI");
}

bool RobotController::getRobotConfigChanged() {
    return _robotConfigChanged;
}

// -------------------------------------------------------------------------------------------------
// Reset helpers
// -------------------------------------------------------------------------------------------------

void RobotController::resetPositions() {
    const size_t n = _robotConfig.numOfSteppers;

    _robotState.jointSteps.assign(n, 0);
    _robotState.targetJointSteps.assign(n, 0);
    _robotState.targetVelocity.assign(n, 0.0f);

    _robotState.currentPosition.assign(6, 0.0f);
    _robotState.targetPosition.assign(6, 0.0f);

    _robotState.rawFerrisValues.clear();
    _robotState.sensorCorrectedJointSteps.assign(n, 0);

    resetSteppers();

    if (_kinematics) {
        updateCurrentPosition();
        updateTargetPosition();
    }
}

void RobotController::resetSteppers() {
    for (auto &stepper : _steppers) {
        stepper.setPosition(0);
        stepper.setSetpointPosition(0);
    }
}

// -------------------------------------------------------------------------------------------------
// Persistence disabled in minimal version
// -------------------------------------------------------------------------------------------------

void RobotController::saveStepperPositions() {
    Serial.println("saveStepperPositions() disabled in minimal version.");
}

void RobotController::loadStepperPositions() {
    Serial.println("loadStepperPositions() disabled in minimal version.");
}

StepperPositions RobotController::getStepperPositions() {
    StepperPositions stepperPos{};

    for (int i = 0; i < 8; i++) {
        stepperPos.jointSteps[i] = 0;
        stepperPos.phaseNrs[i] = 0;
    }

    for (size_t i = 0; i < _steppers.size() && i < 8; i++) {
        stepperPos.jointSteps[i] = _robotState.jointSteps[i];
        stepperPos.phaseNrs[i] = _steppers[i].getPhaseNr();
    }

    return stepperPos;
}

void RobotController::setStepperPositions(StepperPositions &stepperPos) {
    const size_t n = std::min(_steppers.size(), static_cast<size_t>(_robotConfig.degreesOfFreedom));

    for (size_t i = 0; i < n; i++) {
        _robotState.jointSteps[i] = stepperPos.jointSteps[i];
        _robotState.targetJointSteps[i] = stepperPos.jointSteps[i];
        _steppers[i].setPosition(stepperPos.jointSteps[i]);
        _steppers[i].setSetpointPosition(stepperPos.jointSteps[i]);
        _steppers[i].setPhaseNr(stepperPos.phaseNrs[i]);
    }

    if (_kinematics) {
        updateCurrentPosition();
        updateTargetPosition();
    }
}

// -------------------------------------------------------------------------------------------------
// Robot configuration
// -------------------------------------------------------------------------------------------------

void RobotController::setupPinkMamri() {
    // Disabled in minimal version.
    // Keep function only so the header/API can remain stable.
    Serial.println("setupPinkMamri() disabled, using Purple instead.");
    setupPurpleMamri();
}

void RobotController::setupPurpleMamri() {
    _robotConfig.name = RobotName::PURPLE_MAMRI;
    _robotConfig.degreesOfFreedom = 5;
    _robotConfig.numOfSteppers = 5;

    // Disabled in minimal version
    _robotConfig.hasFerrisWheels = false;
    _robotConfig.numOfFerrisWheels = 0;
    _robotConfig.sensorsEnabled = false;

    // Kinematics kept for rad<->steps conversion
    _robotConfig.kinematicsEnabled = false;

    for (int i = 0; i < 8; i++) {
        _robotConfig.valveInverted[i][0] = false;
        _robotConfig.valveInverted[i][1] = false;
    }

    _kinematics.reset();
    _kinematics.reset(new PurpleMamriController());

    _steppers.clear();
    _steppers.reserve(_robotConfig.numOfSteppers);

    for (uint8_t i = 0; i < _robotConfig.numOfSteppers; ++i) {
        PneumaticStepper st = PneumaticStepper::makeTwoCylinderStepper();
        st.setAcceleration(1000);
        st.setDeceleration(1000);
        _steppers.push_back(st);
    }

    const size_t n = _robotConfig.numOfSteppers;
    _robotState.jointSteps.assign(n, 0);
    _robotState.targetJointSteps.assign(n, 0);
    _robotState.targetVelocity.assign(n, 0.0f);
    _robotState.currentPosition.assign(6, 0.0f);
    _robotState.targetPosition.assign(6, 0.0f);
    _robotState.rawFerrisValues.clear();
    _robotState.sensorCorrectedJointSteps.assign(n, 0);

    _robotConfigChanged = true;
}

void RobotController::setupOnlySteppers(int numOfSteppers, int numOfFerrisWheels) {
    (void)numOfSteppers;
    (void)numOfFerrisWheels;
    Serial.println("setupOnlySteppers() disabled, using Purple instead.");
    setupPurpleMamri();
}

void RobotController::noRobotSetup() {
    // Minimal safe empty config
    _robotConfig.name = RobotName::NO_ROBOT;
    _robotConfig.degreesOfFreedom = 0;
    _robotConfig.numOfSteppers = 0;
    _robotConfig.hasFerrisWheels = false;
    _robotConfig.numOfFerrisWheels = 0;
    _robotConfig.sensorsEnabled = false;
    _robotConfig.kinematicsEnabled = false;

    for (int i = 0; i < 8; i++) {
        _robotConfig.valveInverted[i][0] = false;
        _robotConfig.valveInverted[i][1] = false;
    }

    _kinematics.reset();
    _steppers.clear();

    _robotState.jointSteps.clear();
    _robotState.targetJointSteps.clear();
    _robotState.targetVelocity.clear();
    _robotState.currentPosition.assign(6, 0.0f);
    _robotState.targetPosition.assign(6, 0.0f);
    _robotState.rawFerrisValues.clear();
    _robotState.sensorCorrectedJointSteps.clear();

    _robotConfigChanged = true;
}

// -------------------------------------------------------------------------------------------------
// Update / service
// -------------------------------------------------------------------------------------------------

void RobotController::update() {
    // Read current stepper positions into robot state
    for (size_t i = 0; i < _steppers.size(); ++i) {
        _robotState.jointSteps[i] = _steppers[i].getRoundedPosition();
    }

    // Minimal ROS-oriented behavior:
    // always use joint-space position control
    for (size_t i = 0; i < _steppers.size(); ++i) {
        _steppers[i].setSetpointPosition(_robotState.targetJointSteps[i]);
    }

    if (_kinematics) {
        updateCurrentPosition();
        updateTargetPosition();
    }
}

void RobotController::service() {
    bool anyChanged = false;

    for (auto &stepper : _steppers) {
        stepper.work();
        anyChanged |= stepper.testResetRoundedPositionChanged();
    }

    if (anyChanged) {
        turnOnValves();
    }
}

// -------------------------------------------------------------------------------------------------
// Valve state computation
// -------------------------------------------------------------------------------------------------

void RobotController::testValves() {
    for (int i = 0; i < 8; i++) {
        _spi1Manager.writeValves(1 << (i + 8));
        delay(5);
    }
    for (int i = 0; i < 8; i++) {
        _spi1Manager.writeValves(1 << i);
        delay(5);
    }
}

void RobotController::turnOnValves() {
    _valveState = 0x0000;

    for (size_t i = 0; i < _steppers.size(); ++i) {
        for (int c = 0; c < 2; c++) {
            bool newState = _steppers[i].getCylinderState(c) ^ _robotConfig.valveInverted[i][c];
            _valveState |= (static_cast<uint16_t>(newState) << (2 * i + c));
        }
    }

    uint16_t valveBitPattern = ((_valveState & 0x00ff) << 8) | ((_valveState & 0xff00) >> 8);
    _spi1Manager.writeValves(valveBitPattern);
}

void RobotController::turnOffValves() {
    _valveState = 0x0000;
    _spi1Manager.writeValves(0x0000);
}

uint16_t RobotController::getValveState() {
    return _valveState;
}

// -------------------------------------------------------------------------------------------------
// Stepper helpers
// -------------------------------------------------------------------------------------------------

void RobotController::setFrequency(float frequency) {
    for (auto &stepper : _steppers) {
        stepper.setMaxVelocity(frequency);
    }
}

void RobotController::setFrequencyArray(float* frequencyArray) {
    for (size_t i = 0; i < _steppers.size(); ++i) {
        _steppers[i].setMaxVelocity(frequencyArray[i]);
    }
}

float RobotController::getFrequency(uint8_t stepperIndex) {
    if (stepperIndex < _steppers.size()) {
        return _steppers[stepperIndex].getMaxVelocity();
    }
    return -1.0f;
}

PneumaticStepper& RobotController::getStepper(uint8_t stepperIndex) {
    if (!_steppers.empty() && stepperIndex < _steppers.size()) {
        return _steppers[stepperIndex];
    }
    return _steppers[0];
}

// -------------------------------------------------------------------------------------------------
// Joint-space API (this is what ROS will use)
// -------------------------------------------------------------------------------------------------

void RobotController::incrementJointStep(uint8_t jointId, bool direction) {
    if (jointId >= _robotState.jointSteps.size()) {
        return;
    }

    int stepDiff = _robotState.targetJointSteps[jointId] - _robotState.jointSteps[jointId];
    if (stepDiff > -2 && stepDiff < 2) {
        _robotState.targetJointSteps[jointId] += direction ? 1 : -1;
        _jointPosChanged = true;
    }
}

void RobotController::incrementTargetPosition(uint8_t id, bool direction) {
    (void)id;
    (void)direction;
    // Disabled in minimal ROS joint-control version
}

void RobotController::setJointTargetSteps(const std::vector<int> &steps) {
    const size_t n = std::min(steps.size(), _robotState.targetJointSteps.size());

    for (size_t i = 0; i < n; ++i) {
        _robotState.targetJointSteps[i] = steps[i];
    }

    _jointPosChanged = true;
}

void RobotController::setJointTargetStep(size_t idx, int step) {
    if (idx >= _robotState.targetJointSteps.size()) {
        return;
    }

    _robotState.targetJointSteps[idx] = step;
    _jointPosChanged = true;
}

void RobotController::setJointTargetRad(const std::vector<float> &angles) {
    if (!_kinematics) {
        return;
    }

    if (angles.size() < static_cast<size_t>(_robotConfig.degreesOfFreedom)) {
        return;
    }

    std::vector<int> steps = radToSteps(angles);
    if (steps.size() < static_cast<size_t>(_robotConfig.degreesOfFreedom)) {
        return;
    }

    setJointTargetSteps(steps);
}

// -------------------------------------------------------------------------------------------------
// Cartesian API kept as minimal stubs/helpers
// -------------------------------------------------------------------------------------------------

void RobotController::setTargetPosition(const std::vector<float> &pos) {
    if (_robotState.targetPosition.size() < pos.size()) {
        _robotState.targetPosition.resize(pos.size(), 0.0f);
    }

    for (size_t i = 0; i < pos.size(); ++i) {
        _robotState.targetPosition[i] = pos[i];
    }

    _targetPosChanged = true;
}

void RobotController::setTargetPosition(size_t idx, float pos) {
    if (idx >= _robotState.targetPosition.size()) {
        _robotState.targetPosition.resize(idx + 1, 0.0f);
    }

    _robotState.targetPosition[idx] = pos;
    _targetPosChanged = true;
}

void RobotController::setTargetVelocities(const std::vector<float> &velocities) {
    _robotState.targetVelocity = velocities;
}

void RobotController::setTargetVelocity(size_t idx, float velocity) {
    if (idx >= _robotState.targetVelocity.size()) {
        _robotState.targetVelocity.resize(idx + 1, 0.0f);
    }

    _robotState.targetVelocity[idx] = velocity;
}

void RobotController::updateCurrentPosition() {
    if (!_kinematics) {
        return;
    }

    Eigen::Matrix4f H_EE = _kinematics->forwardKinematicsEE(_robotState.jointSteps);
    _robotState.currentPosition = _kinematics->homogenousToPose(H_EE);
}

void RobotController::updateTargetPosition() {
    if (!_kinematics) {
        return;
    }

    Eigen::Matrix4f H_EE = _kinematics->forwardKinematicsEE(_robotState.targetJointSteps);
    _robotState.targetPosition = _kinematics->homogenousToPose(H_EE);
}

std::vector<float> RobotController::stepsToRad(const std::vector<int> &steps) const {
    if (!_kinematics) {
        return {};
    }
    return _kinematics->stepsToRad(steps);
}

std::vector<int> RobotController::radToSteps(const std::vector<float> &angles) const {
    if (!_kinematics) {
        return {};
    }
    return _kinematics->radToSteps(angles);
}

// -------------------------------------------------------------------------------------------------
// State getters
// -------------------------------------------------------------------------------------------------

std::vector<int> RobotController::getSensorCorrectedJointSteps() const {
    return _robotState.sensorCorrectedJointSteps;
}

RobotState RobotController::getRobotState() const {
    return _robotState;
}

RobotConfig RobotController::getRobotConfig() const {
    return _robotConfig;
}

// -------------------------------------------------------------------------------------------------
// Ferris wheels disabled in minimal version
// -------------------------------------------------------------------------------------------------

void RobotController::setFerrisWheelFeedback(const std::vector<float>& sensorValues) {
    (void)sensorValues;
    // Disabled in minimal version
}