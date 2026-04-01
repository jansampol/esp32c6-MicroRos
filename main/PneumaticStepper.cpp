#ifdef ARDUINO
#include <Arduino.h>

#ifdef max
#warning "max macro defined by USB_device.h, but we use <algorithm> max function instead"
#undef max
#undef min
#undef Max
#undef Min
#endif

#ifdef PNEU_DEBUG
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__);
#else
#define DBG_PRINTF(...)
#endif

#else
// Native environment (Windows etc)
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

//#ifdef PIO_UNIT_TESTING
// To facilitate testing
extern unsigned long millis();
extern unsigned long micros();
extern void delay(unsigned long d);
//#endif

#ifdef PNEU_DEBUG
#define DBG_PRINTF(...) printf(__VA_ARGS__);
#else
#define DBG_PRINTF(...)
#endif

#endif

#include "PneumaticStepper.h"
#include <algorithm>

PneumaticStepper::PneumaticStepper(int nCylinder, bool doubleActing, bool triState, int approachDirection, CylinderStrategy cylinderStrategy, float maxVelocity, float position, float setpointPosition, int phaseNr, bool running)
    : doubleActing(doubleActing),
      triState(triState),
      approachDirection(approachDirection),
      cylinderStrategy(cylinderStrategy),
      setpointPosition(setpointPosition),
      maxVelocity(maxVelocity),
      maxAcceleration(DEFAULT_ACCELERATION),
      maxDeceleration(DEFAULT_ACCELERATION),
      lastChangeUs(micros()),
      lastWorkUs(0),
      positionValid(true),
      position(position),
      velocity(0),
      setpointVelocity(0),
      phaseNr(phaseNr),
      running(running),
      floating(false),
      roundedPositionChanged(true),
      lastStepDir(0),
      cylinderState(nCylinder, 0),
      errorCount(0)
{
    updateCylinderState();
}

/*
PneumaticStepper::PneumaticStepper(const PneumaticStepper &other)
    : nCylinder(other.nCylinder),
      doubleActing(other.doubleActing),
      triState(other.triState),
      approachDirection(other.approachDirection),
      cylinderStrategy(other.cylinderStrategy),
      speed(other.speed),
      maxAcceleration(other.maxAcceleration),
      maxDeceleration(other.maxDeceleration),
      lastChangeUs(other.lastChangeUs),
      lastWorkUs(other.lastWorkUs),
      positionValid(other.positionValid),
      position(other.position),
      setpointPosition(other.setpointPosition),
      phaseNr(other.phaseNr),
      running(other.running),
      floating(other.floating),
      _changed(other._changed),
      lastStepDir(other.lastStepDir),
      errorCount(other.errorCount)
{
    for (int i = 0; i < MAX_CYLINDERS; ++i) {
        cylinderState[i] = other.cylinderState[i];
    }
}*/

bool PneumaticStepper::testResetRoundedPositionChanged()
{
    bool tmp = this->roundedPositionChanged;
    this->roundedPositionChanged = false;
    return tmp;
}

void PneumaticStepper::setCylinderStrategy(CylinderStrategy cylinderStrategy)
{
    this->cylinderStrategy = cylinderStrategy;
    restrictSetpoint();
    updateCylinderState();
}

void PneumaticStepper::setSetpointPosition(float setpointPosition)
{
    this->controlStrategy = Controlstrategy::POSITION_CONTROL;
    this->setpointPosition = setpointPosition;
    restrictSetpoint();
}

void PneumaticStepper::restrictSetpoint()
{
    // Calculate final phase based on current phase and position difference
    int stepsDiff = (int)round(setpointPosition - position);
    int finalPhaseNr = (phaseNr + stepsDiff + 2 * getCylinderCount()) % (2 * getCylinderCount());
    if (finalPhaseNr < 0)
    {
        finalPhaseNr += 2 * getCylinderCount();
    }

    // Restrict setpointPosition based on cylinder strategy for single-acting motors
    if (!doubleActing)
    {
        switch (cylinderStrategy)
        {
        case CylinderStrategy::SINGLE_ENGAGE_ONLY:
            if ((finalPhaseNr & 1) == 1)
            {
                setpointPosition -= 1.0f; // Move setpointPosition back one step
            }
            break;
        case CylinderStrategy::DOUBLE_ENGAGE_ONLY:
            if ((finalPhaseNr & 1) == 0)
            {
                setpointPosition += 1.0f; // Move setpointPosition forward one step
            }
            break;
        default:
            break;
        }
    }
}

void PneumaticStepper::setFloating(bool floating)
{
    this->floating = floating;
    if (floating)
    {
        float oldPosition = position;
        position = 0;
        positionValid = false;
        lastStepDir = 0;

        if (roundf(oldPosition) != roundf(position))
        {
            roundedPositionChanged = true;
        }
    }
    updateCylinderState();
}

void PneumaticStepper::setPosition(float newPosition)
{
    if (floating)
    {
        // Do nothing if motor is floating
        return;
    }
    position = newPosition;
    positionValid = true;
    lastStepDir = 0;
    roundedPositionChanged = true; // even if it was not actually changed, we set it to true anyway
    restrictSetpoint();
    updateCylinderState();
}

void PneumaticStepper::setPhaseNr(int newPhaseNr)
{
    int oldPhaseNr = phaseNr;
    phaseNr = newPhaseNr;
    if (oldPhaseNr != phaseNr)
    {
        roundedPositionChanged = true;
        updateCylinderState();
    }
}

void PneumaticStepper::resetLastChangeTime()
{
    lastChangeUs = micros();
}

void PneumaticStepper::updateCylinderState()
{
    if (floating)
    {
        if (!doubleActing)
        {
            // All cylinders down
            for (int i = 0; i < getCylinderCount(); i++)
            {
                cylinderState[i] = 0;
            }
        }
        else if (triState)
        {
            // All cylinders floating
            for (int i = 0; i < getCylinderCount(); i++)
            {
                cylinderState[i] = 2;
            }
        }
        else
        {
            // Non-tri-state double-acting: set all down
            for (int i = 0; i < getCylinderCount(); i++)
            {
                cylinderState[i] = 0;
            }
        }
    }
    else
    {
        // Not floating, use phaseNr
        if (doubleActing)
        {
            if (triState)
            {
                // Set unused cylinders floating
                for (int i = 0; i < getCylinderCount(); i++)
                {
                    cylinderState[i] = 2;
                }
                if (phaseNr < getCylinderCount())
                {
                    cylinderState[phaseNr] = 1;
                }
                else
                {
                    cylinderState[phaseNr - getCylinderCount()] = 0;
                }
            }
            else
            {
                // Double-acting motor: e.g. 2 cylinders: 00 01 11 10. 3 cylinders: 000 001 011 111 110 100.
                for (int i = 0; i < getCylinderCount(); i++)
                {
                    cylinderState[i] = (i < phaseNr) && (phaseNr <= getCylinderCount() + i) ? 1 : 0;
                }
            }
        }
        else
        {
            // Single-acting motor: e.g. 4 cylinders: 0001 0011 0010 0110 0100 1100 1000 1001.
            for (int i = 0; i < getCylinderCount(); i++)
            {
                cylinderState[i] = 0;
                for (int j = -1; j <= 1; j++)
                {
                    if (phaseNr == (2 * i + j + 2 * getCylinderCount()) % (2 * getCylinderCount()))
                    {
                        cylinderState[i] = 1;
                    }
                }
            }
        }
    }
}

void PneumaticStepper::work()
{
    if (controlStrategy == Controlstrategy::POSITION_CONTROL)
    {
        workPositionControl();
    }
    else // VELOCITY_CONTROL
    {
        workVelocityControl();
    }
    
    bool stateValid = true;
    if (cylinderStrategy == CylinderStrategy::DOUBLE_ENGAGE_ONLY && (phaseNr & 1) == 0) {
        stateValid = false;
    }
    if (cylinderStrategy == CylinderStrategy::SINGLE_ENGAGE_ONLY && (phaseNr & 1) == 1) {
        stateValid = false;
    }

    if (stateValid) {
        updateCylinderState();
    }
}

void PneumaticStepper::advancePosition(float oldVelocity, float deltaTime)
{
    // Update position based on current velocity
    //float newPosition = position + 0.5f * (oldVelocity + velocity) * deltaTime;
    float newPosition = position + velocity * deltaTime;
    float positionDelta = newPosition - position;

    if (limitMaxOneStepPerWorkCall) {
        // Limit to maximum 1 step per iteration
        if (positionDelta > 1.0f)
        {
            positionDelta = 1.0f;
            velocity = positionDelta / deltaTime;
            // velocity = maxVelocity;
        }
        else if (positionDelta < -1.0f)
        {
            positionDelta = -1.0f;
            // velocity = -maxVelocity;
            velocity = positionDelta / deltaTime;
        }
    }

    float oldPosition = position;
    position += positionDelta;

    int roundedPositionChange = roundf(position) - roundf(oldPosition);

    if (roundedPositionChange != 0)
    {
        lastStepDir = roundedPositionChange;
        phaseNr = (phaseNr + roundedPositionChange + 2 * getCylinderCount()) % (2 * getCylinderCount());
        roundedPositionChanged = true;
    }
}

void PneumaticStepper::workVelocityControl()
{
    unsigned long timeUs = micros();
    float deltaTime = (lastWorkUs > 0) ? (timeUs - lastWorkUs) * 1e-6f : 0; // Convert to seconds
    lastWorkUs = timeUs;

    if (floating)
    {
        // Motor is floating. We cannot control it, so just set velocity and position to zero.
        position = 0;
        velocity = 0;
    }
    else
    {
        // Accelerate/decelerate towards target velocity
        float deltaVelocity;
        float thisAcceleration;

        // If current velocity and setpoint velocity have same sign and absolute setpoint velocity is bigger than current velocity: motor is accelerating.
        // Otherwise, motor is decelerating.
        if ((std::signbit(getRestrictedSetpointVelocity())==std::signbit(velocity)) && (fabs(getRestrictedSetpointVelocity()) > fabs(velocity))) {
            thisAcceleration = maxAcceleration;
        } else {
            thisAcceleration = maxDeceleration;
        }
        
        if (velocity < getRestrictedSetpointVelocity())
        {
            deltaVelocity = thisAcceleration * deltaTime;
            if (getRestrictedSetpointVelocity() < velocity + deltaVelocity)
            {
                deltaVelocity = getRestrictedSetpointVelocity() - velocity;
            }
        }
        else if (getRestrictedSetpointVelocity() < velocity)
        {
            deltaVelocity = -thisAcceleration * deltaTime;
            if (velocity + deltaVelocity < getRestrictedSetpointVelocity())
            {
                deltaVelocity = getRestrictedSetpointVelocity() - velocity;
            }
        } else {
            deltaVelocity = 0;
        }

        float oldVelocity = velocity;
        velocity += deltaVelocity;
        advancePosition(oldVelocity, deltaTime);
    }
}

void PneumaticStepper::workPositionControl()
{
    unsigned long timeUs = micros();
    float deltaTime = (lastWorkUs > 0) ? (timeUs - lastWorkUs) * 1e-6f : 0; // Convert to seconds
    lastWorkUs = timeUs;

    if (floating)
    {
        // Motor is floating. We cannot control it, so just set velocity and position to zero.
        position = 0;
        velocity = 0;
    }
    else if (maxVelocity < 0)
    {
        // special case: perform a step at every call to work()
        float positionError = getPositionError();
        if (approachDirection != 0 && this->lastStepDir != approachDirection) {
            if (positionError*lastStepDir >= 0) {
                positionError += lastStepDir; // we aim for one step past the original position
                DBG_PRINTF("Adjusting position error to %f based on last step dir %d and approach dir %d\n", positionError, lastStepDir, approachDirection);
            }
        }

        if (positionError > 0)
        {
            position += 1.0f;
            lastStepDir = 1;
            velocity = 1;
            phaseNr = (phaseNr + 1) % (2 * getCylinderCount());
            roundedPositionChanged = true;
        }
        else if (positionError < 0)
        {
            position -= 1.0f;
            lastStepDir = -1;
            velocity = -1;
            phaseNr = (phaseNr - 1 + 2 * getCylinderCount()) % (2 * getCylinderCount());
            roundedPositionChanged = true;
        }
    } 
    else
    {
        // calculate position error based on approximate position after deltaTime
        float positionError = getPositionError(); //setpointPosition - (position+deltaTime*velocity);
        //float positionError = setpointPosition - (position + deltaTime*velocity);

        if (approachDirection != 0 && this->lastStepDir != approachDirection) {
            if (round(positionError)*lastStepDir >= 0) {
                positionError += lastStepDir; // we aim for one step past the original position
                DBG_PRINTF("Adjusting position error to %f based on last step dir %d and approach dir %d\n", positionError, lastStepDir, approachDirection);
            }
        }

        // Calculate target velocity accounting for discrete timestep
        float absTargetVelocity;
        if (running && deltaTime > 0)
        {
            /*
            // Time-based damping: divide error by damping time constant
            // This accounts for discrete timestep and prevents oscillation
            //const float dampingTimeConstant = 0.05f;  // seconds - controls approach speed
            float dampingTimeConstant = fmin(5 * deltaTime, 0.1f);
            absTargetVelocity = fabs(positionError) / (dampingTimeConstant + deltaTime);
            
            // Also respect deceleration limits
            // Calculate maximum velocity allowed by deceleration to reach the target position
            // Factor 0.9 is a temporary hack to work around position overshoot
            float maxAbsDecelerationVelocity = sqrt(2 * maxDeceleration * fabs(positionError));
            absTargetVelocity = std::min(absTargetVelocity, maxAbsDecelerationVelocity);
            */

            float a = maxDeceleration;
            //absTargetVelocity = (a*deltaTime+sqrt((a*deltaTime)*(a*deltaTime)+8*a*fabs(positionError)))/2;
            //absTargetVelocity = sqrt(2*a*fabs(positionError));
            absTargetVelocity = sqrt(2*a*fabs(positionError));
            float velocityReduction = a*deltaTime;

            // do not make absTargetVelocity too small due to velocityReduction
            if (absTargetVelocity-velocityReduction < a*deltaTime) {
                velocityReduction = absTargetVelocity - a*deltaTime;
                if (velocityReduction<0) {
                    velocityReduction = 0;
                }
            }

            /*
            if (velocityReduction > 0.9*absTargetVelocity) {
                velocityReduction = 0.9f*absTargetVelocity;
            }*/

            absTargetVelocity -= velocityReduction;

            if (absTargetVelocity < 0) {
                absTargetVelocity = 0;
            }

            // one-step approach: if we are within one step from the target, then we set target velocity such that we reach the target in one step
            float oneStepVelocity = fabs(positionError) / deltaTime;
            absTargetVelocity = std::min(absTargetVelocity, oneStepVelocity);
        }
        else
        {
            absTargetVelocity = 0;
        }


        float newSetpointVelocity = std::copysign(absTargetVelocity, positionError);
        
        //setpointVelocity = std::clamp(setpointVelocity, -maxVelocity, maxVelocity); // not defined in C++11
        if (newSetpointVelocity > maxVelocity)
        {
            newSetpointVelocity = maxVelocity;
        }
        else if (newSetpointVelocity < -maxVelocity)
        {
            newSetpointVelocity = -maxVelocity;
        }


        {
            float thisAcceleration;
            // If current velocity and setpoint velocity have same sign and absolute setpoint velocity is bigger than current velocity: motor is accelerating.
            // Otherwise, motor is decelerating.
            if ((std::signbit(newSetpointVelocity)==std::signbit(setpointVelocity)) && (fabs(newSetpointVelocity) > fabs(setpointVelocity))) {
                thisAcceleration = maxAcceleration;
            } else {
                thisAcceleration = maxDeceleration;
            }

            // we only change setpointVelocity with at most thisAcceleration to prevent overshoot due to too fast changes in velocity
            float dv = newSetpointVelocity - setpointVelocity;
            if (dv > thisAcceleration * deltaTime)
            {
                setpointVelocity += thisAcceleration * deltaTime;
            }
            else if (dv < -thisAcceleration * deltaTime)
            {
                setpointVelocity -= thisAcceleration * deltaTime;
            }
            else
            {
                setpointVelocity = newSetpointVelocity;
            }
        }


        float oldVelocity = velocity;

        {
            // Accelerate/decelerate towards target velocity
            float deltaVelocity = 0;

            float thisAcceleration;
            if ((std::signbit(setpointVelocity)==std::signbit(velocity)) && (fabs(setpointVelocity) > fabs(velocity))) {
                thisAcceleration = maxAcceleration;
            } else {
                thisAcceleration = maxDeceleration;
            }

            if (setpointVelocity > velocity)
            {
                deltaVelocity = thisAcceleration * deltaTime;
                if (velocity + deltaVelocity > setpointVelocity)
                {
                    deltaVelocity = setpointVelocity - velocity;
                }
            }
            else if (setpointVelocity < velocity)
            {
                deltaVelocity = -thisAcceleration * deltaTime;
                if (velocity + deltaVelocity < setpointVelocity)
                {
                    deltaVelocity = setpointVelocity - velocity;
                }
            }
            velocity += deltaVelocity;
        }

        advancePosition(oldVelocity, deltaTime);
    }

#ifdef PNEU_DEBUG
    printState("d");
#endif
}

float PneumaticStepper::getBrakingPosition() const
{
    if (velocity == 0)
    {
        return position;
    }
    float decel = (velocity > 0) ? -maxDeceleration : maxDeceleration;
    float timeToStop = -velocity / decel;
    float brakingDistance = velocity * timeToStop + 0.5f * decel * timeToStop * timeToStop;
    return position + brakingDistance;
}

#ifdef PIO_UNIT_TESTING
void PneumaticStepper::workUntilSetpointReached()
{
    bool finished = false;
    while (!finished)
    {
        work();
        delay(1);
        if (fabs(getPositionError()) < 1e-6f)
        {
            finished = true;
        }
    }
}

bool PneumaticStepper::workUntilRoundedPositionChanged(float timeoutSec)
{
    float timeStart = millis() * 0.001;
    bool finished = false;
    while (!finished && (millis() * 0.001 - timeStart < timeoutSec))
    {
        delay(10);
        work();
        if (roundedPositionChanged)
        {
            return true;
        }
    }
    return false;
}

void PneumaticStepper::workUntilNoChange()
{
    workUntilRoundedPositionChanged();
}
#endif

void PneumaticStepper::printState(std::string title) const
{
#ifdef ARDUINO
    Serial.print("M-");
    Serial.print(getCylinderCount());
    Serial.print(" tri=");
    Serial.print(triState);
    Serial.print(" strat=");
    switch (cylinderStrategy)
    {
    case CylinderStrategy::SINGLE_ENGAGE_ONLY:
        Serial.print("single only");
        break;
    case CylinderStrategy::SINGLE_ENGAGE_AT_POS:
        Serial.print("single at pos");
        break;
    case CylinderStrategy::DOUBLE_ENGAGE_ONLY:
        Serial.print("double only");
        break;
    case CylinderStrategy::DOUBLE_ENGAGE_AT_POS:
        Serial.print("double at pos");
        break;
    case CylinderStrategy::ANY_ENGAGE:
        Serial.print("any");
        break;
    }
    Serial.print(" maxVelocity=");
    Serial.print(maxVelocity, 2);
    Serial.print(" time=");
    Serial.print(micros());
    Serial.print(" pos=");
    Serial.print(position, 3);
    Serial.print(" set=");
    Serial.print(setpointPosition, 3);
    Serial.print(" speed=");
    Serial.print(velocity, 3);
    Serial.print(" phaseNr=");
    Serial.print(phaseNr);
    Serial.print(" cyl=[");
    for (int i = 0; i < getCylinderCount(); i++)
    {
        Serial.print(cylinderState[i]);
    }
    Serial.print("] err=");
    Serial.print(errorCount);
    Serial.println();
#else
    cout << "\033[34m" << title << "\033[0m ";
    cout << "M-" << getCylinderCount();
    if (triState)
        cout << " tri" << triState;

    if (cylinderStrategy != CylinderStrategy::ANY_ENGAGE) {
        cout << " strat=";
        switch (cylinderStrategy)
        {
        case CylinderStrategy::SINGLE_ENGAGE_ONLY:
            cout << "single only";
            break;
        case CylinderStrategy::SINGLE_ENGAGE_AT_POS:
            cout << "single at pos";
            break;
        case CylinderStrategy::DOUBLE_ENGAGE_ONLY:
            cout << "double only";
            break;
        case CylinderStrategy::DOUBLE_ENGAGE_AT_POS:
            cout << "double at pos";
            break;
        case CylinderStrategy::ANY_ENGAGE:
            cout << "any";
            break;
        }
    }
    cout << " maxVel=" << maxVelocity 
         << " timeSec=" << setprecision(4) << setw(8) << micros() * 0.000001
         << " pos=" << setprecision(6) << setw(8) << position 
         << " set=" << setpointPosition 
         << " vel=" << setprecision(6) << setw(8) << velocity
         << " targetVel=" << setprecision(3) << setw(5) << setpointVelocity
         << " phaseNr=" << setw(1) << phaseNr << " cyl=[";
    for (int i = 0; i < getCylinderCount(); i++)
    {
        cout << (int)cylinderState[i];
    }
    cout << "] ch=" << roundedPositionChanged;
    if (errorCount > 0)
        cout << " err=" << errorCount;
    if (!running)
        cout << " paused";
    cout << endl;
#endif
}

float PneumaticStepper::getRestrictedSetpointVelocity() const
{
    float restrictedVelocity;
    if (running) {
        restrictedVelocity = setpointVelocity;
    } else {
        restrictedVelocity = 0;
    }

    // Restrict velocity to not exceed maxVelocity
    if (restrictedVelocity > maxVelocity)
    {
        restrictedVelocity = maxVelocity;
    }
    else if (restrictedVelocity < -maxVelocity)
    {
        restrictedVelocity = -maxVelocity;
    }

    return restrictedVelocity;
}