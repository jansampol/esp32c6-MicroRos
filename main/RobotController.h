#pragma once

// =============================
// ORIGINAL
// =============================
// #include <Arduino.h>
// #include <Preferences.h>

// =============================
// ESP-IDF / C++ replacements
// =============================
#include <cstdint>
#include <vector>
#include <memory>

// =============================
// REAL STEPPER LIBRARY
// =============================
#include "PneumaticStepper.h"

// =============================
// PROJECT HEADERS
// =============================
#include "RobotStructs.h"
#include "KinematicsController.h"
#include "PurpleMamriController.h"

// =============================
// ORIGINAL
// =============================
// #include "InputController/SPI/SPI1Manager.h"

class RobotController {
public:
    RobotController();

    void begin();
    void changeRobot(RobotName robotName, int numOfSteppersAndFerris = 1);
    void nextRobot();

    bool getRobotConfigChanged();
    void setRobotConfigChanged(bool changed) { _robotConfigChanged = changed; }

    void resetPositions();
    void resetSteppers();

    // Saving and loading robot positions using NVS flash storage
    void saveStepperPositions();
    void loadStepperPositions();
    StepperPositions getStepperPositions();
    void setStepperPositions(StepperPositions &stepperPos);

    // Robot setup
    void setupPinkMamri();
    void setupPurpleMamri();
    void setupOnlySteppers(int numOfSteppers, int numOfFerrisWheels = 0);
    void noRobotSetup();

    void update();
    void service();

    void testValves();
    void turnOffValves();
    void turnOnValves();
    uint16_t getValveState();

    void incrementJointStep(uint8_t jointId, bool direction);
    void incrementTargetPosition(uint8_t id, bool direction);

    RobotState getRobotState() const;
    RobotConfig getRobotConfig() const;

    int getNumOfSteppers() { return _robotConfig.numOfSteppers; }
    int getNumOfFerrisWheels() { return _robotConfig.numOfFerrisWheels; }
    int getDegreesOfFreedom() { return _robotConfig.degreesOfFreedom; }

    // Setters
    void setJointTargetSteps(const std::vector<int> &steps);
    void setJointTargetStep(size_t idx, int step);
    void setJointTargetRad(const std::vector<float> &angles);

    void setTargetPosition(const std::vector<float> &pos);
    void setTargetPosition(size_t idx, float pos);

    void setTargetVelocity(size_t idx, float velocity);
    void setTargetVelocities(const std::vector<float> &velocities);

    void updateCurrentPosition();
    void updateTargetPosition();

    // =============================
    // ORIGINAL
    // =============================
    // void setControlStrategy(PneumaticStepper::Controlstrategy strategy) {
    //     _robotState.controlStrategy = strategy;
    // }

    // ORIGINAL
    void setControlStrategy(PneumaticStepper::Controlstrategy strategy) {
        _robotState.controlStrategy = strategy;
    }

    // TEMP REPLACEMENT
    // void setControlStrategy(ControlStrategy strategy) {
    //     _robotState.controlStrategy = strategy;
    // }
    //    }

    std::vector<float> stepsToRad(const std::vector<int> &steps) const;
    std::vector<int> radToSteps(const std::vector<float> &angles) const;

    std::vector<int> getSensorCorrectedJointSteps() const;

    void setFerrisWheelFeedback(const std::vector<float>& sensorValues);

    void ferrisWheelTareCurrentPosition() {}
    void ferrisWheelResetTare() {}

    void setFrequency(float frequency);
    void setFrequencyArray(float* frequencyArray);
    float getFrequency(uint8_t stepperIndex);
    PneumaticStepper& getStepper(uint8_t stepperIndex);

    void enableIK() { _robotConfig.kinematicsEnabled = true; }
    void disableIK() { _robotConfig.kinematicsEnabled = false; }

    std::unique_ptr<KinematicsController> _kinematics;

private:
    // =============================
    // ORIGINAL
    // =============================
    // SPI1Manager _spi1Manager;

    uint16_t _valveState = 0x0000;

    bool _robotConfigChanged = false;
    bool _targetPosChanged = false;
    bool _jointPosChanged = false;

    RobotConfig _robotConfig;
    RobotState _robotState;

    std::vector<PneumaticStepper> _steppers;

    // =============================
    // ORIGINAL
    // =============================
    // Preferences _preferences;
};