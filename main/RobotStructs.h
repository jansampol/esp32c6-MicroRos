#pragma once

// =============================
// ORIGINAL (Arduino dependency)
// =============================
// #include <Arduino.h>

// =============================
// REPLACEMENT for ESP-IDF
// =============================
#include <cstdint>
#include <vector>

// =============================
// ORIGINAL (Stepper dependency)
// =============================
// #include "PneumaticStepper.h"

// =============================
// NOW ENABLED
// =============================
#include "PneumaticStepper/PneumaticStepper.h"

// =============================
// TEMP REPLACEMENT (kept commented for rollback)
// =============================
// enum class ControlStrategy {
//     POSITION = 0,
//     VELOCITY = 1
// };


// When adding a new robot, add to the next Robotfunction,
// add a config function (copy setupPinkMamri or setupPurpleMamri),
// and add enum conversion to webserver.cpp
enum RobotName {
    NO_ROBOT = -1,
    PINK_MAMRI = 0,
    PURPLE_MAMRI = 1,
    STEPPER_TESTER = 2,
    STEPPER_AND_FERRIS = 3
};


struct RobotConfig {
    RobotName name;

    // assume the user will connect the same amount of ferris wheels as degrees of freedom
    uint8_t degreesOfFreedom;

    // assume num of steppers is always equal to degrees of freedom (end effector is also a stepper)
    uint8_t numOfSteppers;

    bool valveInverted[8][2];

    // whether the robot has the option for ferris wheels.
    bool hasFerrisWheels;
    uint8_t numOfFerrisWheels;

    bool sensorsEnabled;
    bool kinematicsEnabled;
};


// =============================
// Robot State
// =============================
struct RobotState {

    PneumaticStepper::Controlstrategy controlStrategy =
        PneumaticStepper::Controlstrategy::POSITION_CONTROL;

    std::vector<int> jointSteps;
    std::vector<int> targetJointSteps;
    std::vector<float> targetVelocity;

    std::vector<float> currentPosition;
    std::vector<float> targetPosition;

    std::vector<float> rawFerrisValues;
    std::vector<int> sensorCorrectedJointSteps;

    bool needsPositionalFeedback = false;
    bool needsForceFeedback = false;
};


// For saving and loading the robot position on startup
struct StepperPositions {
    int jointSteps[8];
    int phaseNrs[8];
};