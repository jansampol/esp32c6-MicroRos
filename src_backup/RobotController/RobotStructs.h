#pragma once
#include <Arduino.h>
#include <vector>

#include "PneumaticStepper.h"

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

    bool sensorsEnabled; // Whether robot has sensors

    bool kinematicsEnabled;
    // Remember that kinematic specific parameters are set using custom constructor functions in the kinematic class.

};

struct RobotState {
    PneumaticStepper::Controlstrategy controlStrategy;

    // joint control?
    std::vector<int> jointSteps; // {where we are now} --> information is already in _steppers
    std::vector<int> targetJointSteps; // {where we want to go} --> information is already in _steppers
    std::vector<float> targetVelocity; 

    // end effector position control
    std::vector<float> currentPosition; // x, y, z, Rx, Ry, Rz 
    std::vector<float> targetPosition; // x, y, z, Rx, Ry, Rz

    std::vector<float> rawFerrisValues; // raw cumulative position values from ferris wheels
    std::vector<int> sensorCorrectedJointSteps; // sensor-derived joint steps

    // flag bools to activate i2c manager
    bool needsPositionalFeedback = false;
    bool needsForceFeedback = false;
};

// For saving and loading the robot position on startup
struct StepperPositions {
    int jointSteps[8];
    int phaseNrs[8];
};