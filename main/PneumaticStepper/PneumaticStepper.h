#pragma once

#ifdef max
#warning "max macro defined before including PneumaticStepper.h; this may cause compilation errors. Please #undef max before including PneumaticStepper.h"
#endif

#include <stdint.h>
#include <math.h>
#include <vector>
#include <string>

//const int MAX_CYLINDERS = 8;    // increase if using more cylinders;
const float MAX_JITTER = 0.05f; // Timestamps may be off by at most this fraction of one period. This allows to keep a consistent frequency even when timing is slightly off.

class PneumaticStepper
{
public:
    /* Default is position control. 
        Calling setControlStrategy(...) switches between the two modes.
        Calling setSetpointPosition(...) switches to position control mode.
        Calling setSetpointVelocity(...) switches to velocity control mode.
    */
    enum class Controlstrategy
    {
        POSITION_CONTROL,
        VELOCITY_CONTROL
    };

    /*
        In motors with N single-acting cylinder, between 0 and N cylinders may be engaged depending on cylinder state.
        In wedge-driven motors, the drawback is that engaging multiple cylinders simultaneously may stress the teeth of the rack/gear.
        The strategy may be selected to limit the number of simultaneously engaged cylinders, at setpointPosition or at any time.
        The drawback of SINGLE_ENGAGE is that when doing a step, one cylinder retracts and another engages to the rack; during the
        motion the rack might be able to move slightly, causing missed steps.
        For dual-acting cylinders, use ANY_ENGAGE.
    */
    enum class CylinderStrategy
    {
        ANY_ENGAGE,           // default, for dual-acting cylinders
        SINGLE_ENGAGE_ONLY,   // exactly one cylinder is normally engaged
        SINGLE_ENGAGE_AT_POS, // at setpointPosition, only one cylinder may be engaged to the rack/gear
        DOUBLE_ENGAGE_ONLY,   // two cylinders are normally engaged
        DOUBLE_ENGAGE_AT_POS  // at setpointPosition, two cylinders are engaged to the rack/gear
    };

    static constexpr float DEFAULT_ACCELERATION = 100.0; // in steps/sec^2. 100 is slow, 1000 is fast. 1000: In 0.1 second it accelerates from standstill to 100 steps/sec which is almost instant.

    PneumaticStepper(int nCylinder, bool doubleActing, bool triState = false, int approachDirection = 0, CylinderStrategy cylinderStrategy = CylinderStrategy::ANY_ENGAGE,
                     float maxStepFrequency = 10, float position = 0, float setpointPosition = 0, int phaseNr = 0, bool running = true);

    //PneumaticStepper(const PneumaticStepper &other);

    static PneumaticStepper makeTwoCylinderStepper() { return PneumaticStepper(2, true); }
    static PneumaticStepper makeThreeCylinderStepper() { return PneumaticStepper(3, false); }

    int getCylinderCount() const { return cylinderState.size(); } // nCylinder; }
    bool isDoubleActing() const { return doubleActing; }
    bool isTriState() const { return triState; }
    void setApproachDirection(int approachDirection) { this->approachDirection = approachDirection; }
    int getApproachDirection() const { return approachDirection; }
    void setCylinderStrategy(CylinderStrategy cylinderStrategy);
    CylinderStrategy getCylinderStrategy() const { return cylinderStrategy; }

    // velocity=-1 means: perform a step at every call to work()
    void setMaxVelocity(float newMaxVelocity) { maxVelocity = newMaxVelocity; }
    float getMaxVelocity() const { return maxVelocity; }
    [[deprecated("Use setMaxVelocity() instead")]] void setFrequency(float frequency) { setMaxVelocity(frequency); }  // Alias for backward compatibility
    [[deprecated("Use getMaxVelocity() instead")]] float getFrequency() const { return getMaxVelocity(); }  // Alias for backward compatibility
    float getPosition() const { return position; }
    int getRoundedPosition() const { return roundf(position); }

    float getSetpointPosition() const { return setpointPosition; }
    // Also sets strategy to POSITION_CONTROL
    void setSetpointPosition(float setpointPosition);

    float getSetpointVelocity() const { return setpointVelocity; }
    // Also sets strategy to VELOCITY_CONTROL
    void setSetpointVelocity(float velocity) { setpointVelocity = velocity; controlStrategy = Controlstrategy::VELOCITY_CONTROL; }

    [[deprecated("Use getSetpointPosition() instead")]] float getSetpoint() const { return setpointPosition; }
    [[deprecated("Use setSetpointPosition() instead")]] void setSetpoint(float setpointPosition) { setSetpointPosition(setpointPosition); }

    float getVelocity() { return velocity; }
    void setVelocity(float velocity) { this->velocity = velocity; } // directly sets velocity, mostly relevant for velocity control with infinite acceleration
    float getAcceleration() { return maxAcceleration; }
    // Set acceleration. Deceleration is not changed! Use setDeceleration(...) to set deceleration.
    void setAcceleration(float acceleration) { maxAcceleration = acceleration; }
    float getDeceleration() { return maxDeceleration; }
    void setDeceleration(float deceleration) { maxDeceleration = deceleration; }
    bool isPositionValid() const { return positionValid; }
    int getPhaseNr() const { return phaseNr; }
    void setPhaseNr(int phaseNr);
    bool isFloating() const { return floating; }
    // Returns difference between setpointPosition and current position
    [[deprecated("Use getPositionError() instead")]] float getStepsTodo() const { return getPositionError(); }
    float getPositionError() const { return setpointPosition - position; }
    int getLastStepDir() const { return lastStepDir; }
    // Returns byte array with n elements indicating the cylinder states: 0=down, 1=up, 2=floating.
    const std::vector<uint8_t> getCylinderStates() const { return cylinderState; }
    int getCylinderState(int i) { return cylinderState[i]; }

    // Returns true if rounded position (or phase) was changed since the last call to hasRoundedPositionChanged(). Then sets roundedPositionChanged=false.
    bool testResetRoundedPositionChanged();

    // If floating=true: sets cylinder state such that rack or gear can be back-driven. This is always possible in
    // single-acting motors, in tri-state double-acting motors and in motors where the system pressure can be switched off.
    // Setting it to floating invalidates the position, as it can be externally changed.
    // If floating=false: turns floating off but the position is still invalid until the next call to setPosition(...).
    void setFloating(bool newFloating);

    // Sets position and makes it valid but does not change the phase! Does nothing if motor is floating.
    void setPosition(float newPosition);
    float getBrakingPosition() const; // returns position where motor would come to a stop if maximum deceleration is applied immediately

    int getErrorCount() const { return errorCount; }

    bool isRunning() const { return running; }
    void run() { running = true; }
    void pause() { running = false; }

    // Resets last change time to current time, blocking changes to position for the next period
    void resetLastChangeTime();

    // Performs logic, either position control or velocity control.
    // In case of position control: advancing the motor by one step towards the setpoint position if enough time has elapsed since last change
    void work();

#ifdef PIO_UNIT_TESTING
    void workUntilSetpointReached();
    // Returns true if rounded position changed, false if timeout occurred
    bool workUntilRoundedPositionChanged(float timeoutSec=1); // repeatedly calls work() with one ms delay, until rounded position (and thus cylinder state) changes
    [[deprecated("Use workUntilRoundedPositionChanged() instead")]] void workUntilNoChange();
#endif

    // Prints representation to serial (or stdout if non-Arduino)
    void printState(std::string title="") const;

    void setControlStrategy(Controlstrategy strategy) { this->controlStrategy = strategy; }
    Controlstrategy getControlStrategy() const { return controlStrategy; }

    void setLimitMaxOneStepPerWorkCall(bool limit) { this->limitMaxOneStepPerWorkCall = limit; }
    bool getLimitMaxOneStepPerWorkCall() const { return limitMaxOneStepPerWorkCall; }
private:
    // Helper methods
    void updateCylinderState();
    void restrictSetpoint();

    void workPositionControl();
    void workVelocityControl();
    void advancePosition(float oldVelocity, float deltaTime);

    // returns setpointVelocity restricted to not exceed maxVelocity and such that the setpointPosition is approached in the correct direction
    float getRestrictedSetpointVelocity() const;

    Controlstrategy controlStrategy = Controlstrategy::POSITION_CONTROL;
    //int nCylinder; use cylinderState.size() instead
    bool doubleActing;
    bool triState;
    int approachDirection;
    CylinderStrategy cylinderStrategy;

    float setpointPosition; // in steps
    float maxVelocity;      // in steps/sec, positive. If -1, then perform a step at every call to work()
    float maxAcceleration;  // in steps/sec^2
    float maxDeceleration;  // in steps/sec^2, usually larger than maxAcceleration because slowing down is easy

    unsigned long lastChangeUs; // timestamp of last change
    unsigned long lastWorkUs;   // timestamp of last work() routine
    bool positionValid;         // normally true, changes to false when floating and changes back to true when calibrating current position

    // state variables
    float position;
    float velocity; // current velocity in steps per second, positive or negative
    float setpointVelocity; // target velocity in steps per second, positive or negative. Used in both position control and velocity control.

    int phaseNr;
    bool running; // if not running then motor is decelerated to stop

    /*
     * There are 2N phases (but in single-acting motors with CylinderStrategy SINGLE_ENGAGE_ONLY and DOUBLE_ENGAGE_ONLY, half of the phases are skipped)
     * Single-acting motor: 3 cylinders: 001 011 010 110 100 101. 4 cylinders: 0001 0011 0010 0110 0100 1100 1000 1001. 5 cylinders: 00001 00011 etc.
     * Double-acting motor: 2 cylinders: 00 01 11 10. 3 cylinders: 000 001 011 111 110 100. 4 cylinders: 0000 0001 0011 0111 1111 1110 1100 1000. 5 cylinders: 00000	etc.
     */
    bool floating; // if true then one or more cylinders are in an undetermined, "floating" state. Only for motors with tri-state cylinders or single-acting cylinders.
    bool roundedPositionChanged;  // Is guaranteed to be set when rounded position is changed. Sometimes also when it was not actually changed (such as in setPosition()).

    // Direction of last step, can be -1, or 1 if it is known, and 0 if unknown.
    // Is used by work() to ensure that the setpointPosition is approached in the correct direction.
    int lastStepDir;

    // The state of each cylinder, is linked to _phaseNr and _floating. 0=down, 1=up, 2=floating (only for tri-state cylinders).
    std::vector<uint8_t> cylinderState;//[MAX_CYLINDERS];

    int errorCount;

    bool limitMaxOneStepPerWorkCall = true; // if true, then at most one step is performed per call to work(). This is needed to ensure that the timing of steps is correct, but may be set to false for testing or if the work() routine is called very frequently.
};