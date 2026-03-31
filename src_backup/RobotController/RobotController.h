/*
    This is the old MAMRI class.

    It is updated so it is split between an .h file and .cpp file.
    It references the PneumaticStepper class, and has a KinematicsController as a child
    The SPI1 manager is used to update the motor valves, and is a wrapper for the MCP23S17 class.
    This is done for consistency with the other SPI bus. (This one is without CS)

    The inputcontroller communicates with this object via set and get functions.
    Making use of the RobotState struct.

    Using the function changeRobot(RobotName robotName) the robot can be switched at runtime.
    The robot config structs are set in their own functions, remember that kinematic specific
    parameters are set in the KinematicsController class.

    If a robot does not have any kinematics, it can be disabled by setting the value .kinematicsEnabled to false.
*/

#pragma once
#include <Arduino.h>
// #include <Eigen/Dense>
#include <Preferences.h>

#include "PneumaticStepper.h"
#include <vector>
#include <memory>

// Robot state & config
#include "SystemParameters.h"
#include "RobotController/RobotStructs.h"
#include "RobotController/KinematicsController.h"
#include "RobotController/Robots/PurpleMamriController.h"

#include "InputController/SPI/SPI1Manager.h"

class RobotController {
    public:

        // Constructor
        RobotController();
        void begin();
        void changeRobot(RobotName robotName, int numOfSteppersAndFerris = 1);
        void nextRobot();
        
        bool getRobotConfigChanged();
        void setRobotConfigChanged(bool changed){ _robotConfigChanged = changed; }

        void resetPositions();
        void resetSteppers();

        // Saving and loading robot positions using NVS flash storage
        void saveStepperPositions();
        void loadStepperPositions();
        StepperPositions getStepperPositions();
        void setStepperPositions(StepperPositions &stepperPos);

        // to switch robots
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


        // can increment either joint steps, or target position of end effector, based on control mode
        void incrementJointStep(uint8_t jointId, bool direction);
        void incrementTargetPosition(uint8_t id, bool direction);
        
        RobotState getRobotState() const;
        
        RobotConfig getRobotConfig() const;
        int getNumOfSteppers(){ return _robotConfig.numOfSteppers; }
        int getNumOfFerrisWheels(){ return _robotConfig.numOfFerrisWheels; }
        int getDegreesOfFreedom(){ return _robotConfig.degreesOfFreedom; }

        // Setters: allow replacing the full target lists or single entries
        void setJointTargetSteps(const std::vector<int> &steps);
        void setJointTargetStep(size_t idx, int step);
        void setJointTargetRad(const std::vector<float> &angles);
        void setTargetPosition(const std::vector<float> &pos);
        void setTargetPosition(size_t idx, float pos);
        
        // Velocity control setters
        void setTargetVelocity(size_t idx, float velocity);
        void setTargetVelocities(const std::vector<float> &velocities);

        // Update current position based on joint steps
        void updateCurrentPosition();
        // Update target position based on target joint steps
        void updateTargetPosition();

        void setControlStrategy(PneumaticStepper::Controlstrategy strategy) {_robotState.controlStrategy = strategy;}

        // Joint steps and joint angles conversions
        // Currently calls the kinematics controller functions
        std::vector<float> stepsToRad(const std::vector<int> &steps) const;
        std::vector<int> radToSteps(const std::vector<float> &angles) const;

        std::vector<int> getSensorCorrectedJointSteps() const;
        
        // can have a runtime change of number of ferris wheels. use a span to get a reference to a vector with read values
        void setFerrisWheelFeedback(const std::vector<float>& sensorValues);

        // Ferris wheel sensor tare/zeroing
        void ferrisWheelTareCurrentPosition();
        void ferrisWheelResetTare();

        // mamri class specific functions
        void setFrequency(float frequency);
        void setFrequencyArray(float* frequencyArray);
        float getFrequency(uint8_t stepperIndex);
        PneumaticStepper& getStepper(uint8_t stepperIndex);

        void enableIK() {_robotConfig.kinematicsEnabled = true;}
        void disableIK() {_robotConfig.kinematicsEnabled = false;}

        // smart pointer to kinematics controller object (deletion handled automatically)
        std::unique_ptr<KinematicsController> _kinematics;

    private:
        SPI1Manager _spi1Manager;

        uint16_t _valveState = 0x0000;

        bool _robotConfigChanged = false;
        bool _targetPosChanged = false;
        bool _jointPosChanged = false;

        RobotConfig _robotConfig;
        RobotState _robotState;
        std::vector<PneumaticStepper> _steppers;
        Preferences _preferences;
};

