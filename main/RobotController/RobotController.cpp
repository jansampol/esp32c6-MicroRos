#include "RobotController.h"

#include <algorithm>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SystemParameters.h"

static const char *TAG = "RobotController";

namespace {
constexpr float kFerrisToJointScale[] = {
    5.0f / 8.0f,
    5.0f / 9.5f,
    5.0f / 4.0f,
    5.0f / 4.0f,
   -5.0f / 4.5f,
};

float ferrisToJointScale(size_t jointIdx) {
    constexpr size_t scaleCount = sizeof(kFerrisToJointScale) / sizeof(kFerrisToJointScale[0]);
    return (jointIdx < scaleCount) ? kFerrisToJointScale[jointIdx] : 1.0f;
}
}

RobotController::RobotController() :
    _kinematics()
{
}

void RobotController::begin() {

    #if ACTIVE_SPI_RUNTIME_MODE == SPI_RUNTIME_MODE_SPI1_ONLY
    if (!_spi1Manager.begin()) {
        ESP_LOGE(TAG, "SPI1Manager begin() failed; motor valve writes will not reach hardware");
    }
    turnOffValves();
    _valveState = 0x0000;
    #else
    // In SPI0-only mode we must not touch SPI1.
    _valveState = 0x0000;
    #endif

    setupPurpleMamri();
    resetSteppers();
    resetPositions();

    ESP_LOGI(TAG, "Switched to PURPLE_MAMRI");
}

// -------------------------------------------------------------------------------------------------
// Minimal robot selection API
// -------------------------------------------------------------------------------------------------

void RobotController::nextRobot() {
    changeRobot(PURPLE_MAMRI);
}

void RobotController::changeRobot(RobotName robotName, int numOfSteppersAndFerris) {
    (void)robotName;
    (void)numOfSteppersAndFerris;

    setupPurpleMamri();
    ESP_LOGI(TAG, "Switched to PURPLE_MAMRI");
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
    _robotState.ferrisWheelJointSteps.assign(n, 0);

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
// Persistence disabled in current version
// -------------------------------------------------------------------------------------------------

void RobotController::saveStepperPositions() {
    // =============================
    // ORIGINAL
    // =============================
    // save to Preferences / NVS

    ESP_LOGI(TAG, "saveStepperPositions() disabled in current version.");
}

void RobotController::loadStepperPositions() {
    // =============================
    // ORIGINAL
    // =============================
    // load from Preferences / NVS

    ESP_LOGI(TAG, "loadStepperPositions() disabled in current version.");
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

    ESP_LOGI(TAG, "setupPinkMamri() disabled, using Purple instead.");
    setupPurpleMamri();
}

void RobotController::setupPurpleMamri() {
    _robotConfig.name = RobotName::PURPLE_MAMRI;
    _robotConfig.degreesOfFreedom = 5;
    _robotConfig.numOfSteppers = 6;

    _robotConfig.hasFerrisWheels = false;
    _robotConfig.numOfFerrisWheels = 0;
    _robotConfig.sensorsEnabled = false;

    // Kinematics enabled
    _robotConfig.kinematicsEnabled = true;

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
    _robotState.ferrisWheelJointSteps.assign(n, 0);

    _robotConfigChanged = true;
}

void RobotController::setupOnlySteppers(int numOfSteppers, int numOfFerrisWheels) {
    (void)numOfSteppers;
    (void)numOfFerrisWheels;

    ESP_LOGI(TAG, "setupOnlySteppers() disabled, using Purple instead.");
    setupPurpleMamri();
}

void RobotController::noRobotSetup() {
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
    _robotState.ferrisWheelJointSteps.clear();

    _robotConfigChanged = true;
}

// -------------------------------------------------------------------------------------------------
// Update / service
// -------------------------------------------------------------------------------------------------

void RobotController::update() {
    for (size_t i = 0; i < _steppers.size(); ++i) {
        _robotState.jointSteps[i] = _steppers[i].getRoundedPosition();
    }

    for (size_t i = 0; i < _steppers.size(); ++i) {
        if (_robotState.controlStrategy == PneumaticStepper::Controlstrategy::VELOCITY_CONTROL) {
            const float velocity = (i < _robotState.targetVelocity.size()) ? _robotState.targetVelocity[i] : 0.0f;
            _steppers[i].setSetpointVelocity(velocity);
        } else {
            _steppers[i].setSetpointPosition(_robotState.targetJointSteps[i]);
        }
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

    static uint32_t log_counter = 0;
    ++log_counter;

    if (log_counter >= 500) {
        log_counter = 0;

        for (size_t i = 0; i < _robotState.jointSteps.size(); ++i) {
            ESP_LOGI(
                TAG,
                "joint[%u] current_steps=%d target_steps=%d target_vel=%.2f",
                (unsigned)i,
                _robotState.jointSteps[i],
                _robotState.targetJointSteps[i],
                (i < _robotState.targetVelocity.size()) ? _robotState.targetVelocity[i] : 0.0f
            );
        }


        ESP_LOGI(TAG,"valve_state=0x%04x", _valveState);
    }
}

// -------------------------------------------------------------------------------------------------
// Valve state computation
// -------------------------------------------------------------------------------------------------

void RobotController::testValves() {
    // =============================
    // ORIGINAL
    // =============================
    // SPI1 valve line test
    for (int i = 0; i < 8; i++) {
        _spi1Manager.writeValves(1 << (i + 8));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    for (int i = 0; i < 8; i++) {
        _spi1Manager.writeValves(1 << i);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "testValves() completed");
}

void RobotController::turnOnValves() {
    _valveState = 0x0000;

    for (size_t i = 0; i < _steppers.size(); ++i) {
        for (int c = 0; c < 2; c++) {
            bool newState = _steppers[i].getCylinderState(c) ^ _robotConfig.valveInverted[i][c];
            _valveState |= (static_cast<uint16_t>(newState) << (2 * i + c));
        }
    }

    // SPI1 valve output write
    _spi1Manager.writeValves(_valveState);
}

void RobotController::turnOffValves() {
    _valveState = 0x0000;
    // SPI1 valve output write
    _spi1Manager.writeValves(0x0000);

    _valveState = 0x0000;
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
    return _steppers[stepperIndex];
}

// -------------------------------------------------------------------------------------------------
// Joint-space API
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
}

void RobotController::setJointTargetSteps(const std::vector<int> &steps) {
    const size_t n = std::min(steps.size(), _robotState.targetJointSteps.size());

    _robotState.controlStrategy = PneumaticStepper::Controlstrategy::POSITION_CONTROL;

    for (size_t i = 0; i < n; ++i) {
        _robotState.targetJointSteps[i] = steps[i];
    }

    _jointPosChanged = true;
}

void RobotController::setJointTargetStep(size_t idx, int step) {
    if (idx >= _robotState.targetJointSteps.size()) {
        return;
    }

    _robotState.controlStrategy = PneumaticStepper::Controlstrategy::POSITION_CONTROL;
    _robotState.targetJointSteps[idx] = step;
    _jointPosChanged = true;
}

void RobotController::setNeedleIncisionTargetSteps(int step) {
    if (_robotState.targetJointSteps.empty()) {
        return;
    }

    const size_t needleJointIdx = _robotState.targetJointSteps.size() - 1;
    _robotState.targetJointSteps[needleJointIdx] = step;
}

void RobotController::setJointTargetRad(const std::vector<float> &angles) {
    if (!_kinematics) {
        ESP_LOGW(TAG, "No kinematics controller available");
        return;
    }

    if (angles.size() < static_cast<size_t>(_robotConfig.degreesOfFreedom)) {
        ESP_LOGW(TAG, "Not enough joint angles: got %u expected %u",
                 (unsigned)angles.size(),
                 (unsigned)_robotConfig.degreesOfFreedom);
        return;
    }

    std::vector<int> steps = radToSteps(angles);
    if (steps.size() < static_cast<size_t>(_robotConfig.degreesOfFreedom)) {
        ESP_LOGW(TAG, "radToSteps() returned invalid size");
        return;
    }

    //setJointTargetSteps(steps);
    setSynchronizedJointTargetSteps(steps, 10.0f);

    for (size_t i = 0; i < steps.size(); ++i) {
        ESP_LOGI(TAG, "target joint[%u]: rad=%.6f -> steps=%d",
                 (unsigned)i, angles[i], steps[i]);
    }
}

// This sets the target joint steps and also adjusts the max velocity of each stepper so that they reach the target at the same time.

void RobotController::setSynchronizedJointTargetSteps(const std::vector<int>& steps, float baseMaxVelocity)
{
    const size_t n = std::min(steps.size(), _robotState.jointSteps.size());
    if (n == 0) return;

    _robotState.controlStrategy = PneumaticStepper::Controlstrategy::POSITION_CONTROL;

    int maxDistance = 0;
    std::vector<int> distances(n, 0);

    for (size_t i = 0; i < n; ++i) {
        distances[i] = std::abs(steps[i] - _robotState.jointSteps[i]);
        if (distances[i] > maxDistance) {
            maxDistance = distances[i];
        }
    }

    if (maxDistance == 0) {
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        _robotState.targetJointSteps[i] = steps[i];

        float ratio = static_cast<float>(distances[i]) / static_cast<float>(maxDistance);
        float v = baseMaxVelocity * ratio;

        // avoid zero or too small velocity for tiny moves
        const float minVelocity = 1.0f;
        if (distances[i] > 0 && v < minVelocity) {
            v = minVelocity;
        }
        if (distances[i] == 0) {
            v = minVelocity;
        }

        _steppers[i].setMaxVelocity(v);
    }

    _jointPosChanged = true;
}

// Debug / manual homing API

void RobotController::jogJointSteps(size_t jointIdx, int deltaSteps) {
    if (jointIdx >= _robotState.targetJointSteps.size()) {
        ESP_LOGW(TAG, "jogJointSteps: invalid joint index %u", (unsigned)jointIdx);
        return;
    }

    const int maxJogStep = 10;
    if (deltaSteps > maxJogStep) {
        deltaSteps = maxJogStep;
    } else if (deltaSteps < -maxJogStep) {
        deltaSteps = -maxJogStep;
    }

    _robotState.controlStrategy = PneumaticStepper::Controlstrategy::POSITION_CONTROL;
    _robotState.targetJointSteps[jointIdx] += deltaSteps;
    _jointPosChanged = true;

    ESP_LOGI(
        TAG,
        "Jog joint[%u] by %+d -> new target=%d",
        (unsigned)jointIdx,
        deltaSteps,
        _robotState.targetJointSteps[jointIdx]
    );
}

void RobotController::tareJointToZero(size_t jointIdx) {
    if (jointIdx >= _steppers.size() || jointIdx >= _robotState.jointSteps.size() ||
        jointIdx >= _robotState.targetJointSteps.size()) {
        ESP_LOGW(TAG, "tareJointToZero: invalid joint index %u", (unsigned)jointIdx);
        return;
    }

    // Make current physical position become software zero
    _steppers[jointIdx].setPosition(0);
    _steppers[jointIdx].setSetpointPosition(0);

    _robotState.jointSteps[jointIdx] = 0;
    _robotState.targetJointSteps[jointIdx] = 0;

    _jointPosChanged = true;

    if (_kinematics) {
        updateCurrentPosition();
        updateTargetPosition();
    }

    ESP_LOGI(TAG, "Joint[%u] tared to zero", (unsigned)jointIdx);
}

void RobotController::sendJointToHome(size_t jointIdx) {
    if (jointIdx >= _robotState.targetJointSteps.size()) {
        ESP_LOGW(TAG, "sendJointToHome: invalid joint index %u", (unsigned)jointIdx);
        return;
    }

    _robotState.targetJointSteps[jointIdx] = 0;
    _jointPosChanged = true;

    if (_kinematics) {
        updateTargetPosition();
    }

    ESP_LOGI(TAG, "Joint[%u] commanded to home (target=0)", (unsigned)jointIdx);
}

void RobotController::sendAllJointsToHome() {
    for (size_t i = 0; i < _robotState.targetJointSteps.size(); ++i) {
        _robotState.targetJointSteps[i] = 0;
    }

    _jointPosChanged = true;

    if (_kinematics) {
        updateTargetPosition();
    }

    ESP_LOGI(TAG, "All joints commanded to home (target=0)");
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

int RobotController::needleDepthMmToSteps(float depthMm) const {
    if (!_kinematics) {
        return 0;
    }
    return _kinematics->needleDepthMmToSteps(depthMm);
}

float RobotController::needleStepsToDepthMm(int steps) const {
    if (!_kinematics) {
        return 0.0f;
    }
    return _kinematics->needleStepsToDepthMm(steps);
}

// -------------------------------------------------------------------------------------------------
// State getters
// -------------------------------------------------------------------------------------------------

std::vector<int> RobotController::getFerrisWheelJointSteps() const {
    return _robotState.ferrisWheelJointSteps;
}

RobotState RobotController::getRobotState() const {
    return _robotState;
}

RobotConfig RobotController::getRobotConfig() const {
    return _robotConfig;
}

// -------------------------------------------------------------------------------------------------
// Ferris wheels disabled in current version
// -------------------------------------------------------------------------------------------------

void RobotController::setFerrisWheelFeedback(const std::vector<float>& sensorValues) {
    _robotState.rawFerrisValues = sensorValues;
    
    if (!_kinematics || sensorValues.size() < _robotConfig.degreesOfFreedom) {
        return;
    }

    const float degreesToRadians = PI_FLOAT / 180.0f;
    std::vector<float> taredSensorValuesDeg;
    std::vector<float> jointSensorValuesDeg;
    std::vector<float> jointAnglesRad;
    taredSensorValuesDeg.reserve(_robotConfig.degreesOfFreedom);
    jointSensorValuesDeg.reserve(_robotConfig.degreesOfFreedom);
    jointAnglesRad.reserve(_robotConfig.degreesOfFreedom);

    // Apply zero offset, convert Ferris angle to joint angle, then convert to radians.
    for (size_t i = 0; i < static_cast<size_t>(_robotConfig.degreesOfFreedom); ++i) {
        float offsetSensorValue = sensorValues[i];
        if (i < _ferrisWheelZeroOffset.size()) {
            offsetSensorValue -= _ferrisWheelZeroOffset[i];
        }
        const float jointSensorValue = offsetSensorValue * ferrisToJointScale(i);
        taredSensorValuesDeg.push_back(offsetSensorValue);
        jointSensorValuesDeg.push_back(jointSensorValue);
        jointAnglesRad.push_back(jointSensorValue * degreesToRadians);
    }

    std::vector<int> steps = radToSteps(jointAnglesRad);
    _robotState.ferrisWheelJointSteps = steps;

    // Keep the Ferris wheel joint-step vector aligned with the joint state size if needed.
    if (_robotState.ferrisWheelJointSteps.size() < _robotState.jointSteps.size()) {
        _robotState.ferrisWheelJointSteps.resize(_robotState.jointSteps.size(), 0);
    }

    // filter every 15 loop cycles to reduce log spam, and only if we have valid data
    static int log_counter = 0;
    log_counter++;
    // Print the output in steps for debugging
    if (log_counter >= 40) {
        for (size_t i = 0; i < jointSensorValuesDeg.size(); ++i) {
            ESP_LOGI(TAG, "Ferris wheel joint[%u]: tared_sensor=%.2f deg scale=%.3f joint_sensor=%.2f deg",
                     (unsigned)i,
                     taredSensorValuesDeg[i],
                     ferrisToJointScale(i),
                     jointSensorValuesDeg[i]);
        }
        log_counter = 0;
    }

    _robotState.needsPositionalFeedback = true;
}

void RobotController::ferrisWheelTareCurrentPosition() {
    // Validate: only tare if we've received sensor data
    if (_robotState.rawFerrisValues.empty()) {
        ESP_LOGW(TAG, "Cannot tare Ferris wheels: no sensor data received yet");
        return;
    }
    
    // Capture current sensor readings as the zero baseline
    _ferrisWheelZeroOffset = _robotState.rawFerrisValues;
    
    // Ensure offset vector is sized correctly
    if (_ferrisWheelZeroOffset.size() < static_cast<size_t>(_robotConfig.degreesOfFreedom)) {
        _ferrisWheelZeroOffset.resize(_robotConfig.degreesOfFreedom, 0.0f);
    }
    
    ESP_LOGI(TAG, "Ferris wheels tared at:");
    for (size_t i = 0; i < _ferrisWheelZeroOffset.size(); ++i) {
        ESP_LOGI(TAG, "  Joint[%u]: %.2f degrees", (unsigned)i, _ferrisWheelZeroOffset[i]);
    }
}

void RobotController::setNewPath(const std::vector<std::vector<float>> &path, size_t path_waypoints, size_t path_dof) {
    _path = path;
    _pathWaypoints = path_waypoints;
    _pathDof = path_dof;
    _currentWaypoint = 0;
    _pathExecuting = true;
    _waypointSent = false;

    ESP_LOGI(TAG, "New path accepted: %zu waypoints, %zu DOF", path_waypoints, path_dof);
}

void RobotController::processMotionControl(bool executing_path, size_t path_waypoints, size_t path_dof) {
    if (!executing_path || path_waypoints == 0 || path_dof == 0) {
        return;
    }

    // Send waypoint if not already sent
    if (!_waypointSent && _currentWaypoint < _pathWaypoints && !_path.empty()) {
        if (_currentWaypoint < _path.size()) {
            std::vector<float> target(_path[_currentWaypoint].begin(), _path[_currentWaypoint].end());
            setJointTargetRad(target);
            _waypointSent = true;

            ESP_LOGI(TAG, "Sent waypoint %zu / %zu",
                     _currentWaypoint + 1, _pathWaypoints);
        }
    }

    // Check if waypoint reached and advance
    if (_waypointSent && isAtStepTarget()) {
        ESP_LOGI(TAG, "Reached waypoint %zu / %zu",
                 _currentWaypoint + 1, _pathWaypoints);
        _currentWaypoint++;
        _waypointSent = false;

        if (_currentWaypoint >= _pathWaypoints) {
            ESP_LOGI(TAG, "Path execution finished");
        }
    }

    // To start implementing the Close Loop controller, we will start printing the 
    // different errors (Ferris wheel feedback vs expected current position) here for debugging and analysis.
    static uint32_t error_log_counter = 0;
    ++error_log_counter;
    
    if (error_log_counter >= 20) {  // Log every 20 cycles to reduce spam
        error_log_counter = 0;
        for (size_t i = 0; i < std::min(_robotState.ferrisWheelJointSteps.size(), _robotState.targetJointSteps.size()); ++i) {
            float tairedFerrisValue = 0.0f;
            if (i < _robotState.rawFerrisValues.size() && i < _ferrisWheelZeroOffset.size()) {
                tairedFerrisValue = _robotState.rawFerrisValues[i] - _ferrisWheelZeroOffset[i];
            }
            int error = _robotState.ferrisWheelJointSteps[i] - _robotState.targetJointSteps[i];
            ESP_LOGI(TAG, "Joint[%u] current_steps=%d target=%d steps ferris_tared=%.2f deg ferris_steps=%d error=%+d steps",
                     (unsigned)i,
                     _robotState.jointSteps[i],
                     _robotState.targetJointSteps[i],
                     tairedFerrisValue,
                     _robotState.ferrisWheelJointSteps[i],
                     error);
        }
    }

    /*
    // Closed-loop motion-control strategy - disabled until Ferris wheel feedback is calibrated.
    //
    // Goal:
    // Keep the existing waypoint and synchronized open-loop motion, but use Ferris wheel feedback
    // to detect missed steps, adapt synchronization, and verify waypoint completion from measured
    // joint position instead of commanded step position.
    //
    // Position definitions:
    // q_ref_steps    = _robotState.targetJointSteps      // next waypoint / motion objective
    // q_ol_steps     = _robotState.jointSteps            // open-loop step estimate
    // q_sensor_steps = calibrated Ferris wheel feedback  // measured physical joint position

    static constexpr int CL_WAYPOINT_TOLERANCE_STEPS = 3;
    static constexpr int CL_CORRECTION_THRESHOLD_STEPS = 8;
    static constexpr int CL_RECOVERY_THRESHOLD_STEPS = 30;
    static constexpr int CL_MAX_RECOVERY_ATTEMPTS = 3;
    static constexpr float CL_BASE_MAX_VELOCITY = 10.0f;

    if (_robotState.ferrisWheelJointSteps.size() >= _robotState.targetJointSteps.size()) {
        const size_t n = std::min(_robotState.targetJointSteps.size(), _robotState.jointSteps.size());

        bool waypointReachedBySensor = true;
        bool needsAdaptiveCorrection = false;
        bool needsRecovery = false;
        size_t recoveryJoint = 0;
        int largestSlipError = 0;

        std::vector<int> remainingTargetSteps(n, 0);

        for (size_t i = 0; i < n; ++i) {
            const int qRef = _robotState.targetJointSteps[i];
            const int qOl = _robotState.jointSteps[i];
            const int qSensor = _robotState.ferrisWheelJointSteps[i];

            const int slipError = qSensor - qOl;
            const int targetError = qRef - qSensor;
            remainingTargetSteps[i] = qRef;

            if (std::abs(targetError) > CL_WAYPOINT_TOLERANCE_STEPS) {
                waypointReachedBySensor = false;
            }

            if (std::abs(slipError) > CL_CORRECTION_THRESHOLD_STEPS) {
                needsAdaptiveCorrection = true;
            }

            if (std::abs(slipError) > std::abs(largestSlipError)) {
                largestSlipError = slipError;
                recoveryJoint = i;
            }

            if (std::abs(slipError) > CL_RECOVERY_THRESHOLD_STEPS) {
                needsRecovery = true;
            }
        }

        // Waypoint verification should eventually use measured sensor position.
        if (_waypointSent && waypointReachedBySensor) {
            ESP_LOGI(TAG, "Reached waypoint %zu / %zu using Ferris feedback",
                     _currentWaypoint + 1, _pathWaypoints);
            _currentWaypoint++;
            _waypointSent = false;

            if (_currentWaypoint >= _pathWaypoints) {
                ESP_LOGI(TAG, "Path execution finished");
            }
        }

        if (needsRecovery) {
            static int recoveryAttempts = 0;
            recoveryAttempts++;

            if (recoveryAttempts > CL_MAX_RECOVERY_ATTEMPTS) {
                ESP_LOGE(TAG, "Closed-loop recovery failed; stopping motion");
                for (size_t i = 0; i < n; ++i) {
                    _steppers[i].setMaxVelocity(0.0f);
                    _robotState.targetJointSteps[i] = _robotState.jointSteps[i];
                }
                _pathExecuting = false;
                _waypointSent = false;
                return;
            }

            // Pause all other joints and command only the joint with the largest slip error.
            for (size_t i = 0; i < n; ++i) {
                if (i == recoveryJoint) {
                    _steppers[i].setMaxVelocity(CL_BASE_MAX_VELOCITY);
                    _robotState.targetJointSteps[i] = remainingTargetSteps[i];
                } else {
                    _steppers[i].setMaxVelocity(0.0f);
                    _robotState.targetJointSteps[i] = _robotState.jointSteps[i];
                }
            }
            return;
        }

        if (needsAdaptiveCorrection) {
            // Recompute synchronized motion using the sensor-measured remaining distance.
            // The implementation should convert targetError into new joint targets/frequencies
            // so all joints still arrive at the waypoint together.
            setSynchronizedJointTargetSteps(remainingTargetSteps, CL_BASE_MAX_VELOCITY);
        }
    }
    */
}


bool RobotController::isAtStepTarget() const
{
    const size_t n = std::min(_robotState.jointSteps.size(), _robotState.targetJointSteps.size());
    for (size_t i = 0; i < n; ++i) {
        if (_robotState.jointSteps[i] != _robotState.targetJointSteps[i]) {
            return false;
        }
    }
    return true;
}
