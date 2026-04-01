/*
	ServoValve library
	Represents one servo-controlled pneumatic valve with three states (low, high and neutral).
	It manages the servo angle based on logical input and current time.
	It takes care of the hysteresis / undershoot issue caused by friction in the valve which prevents the servo from fully reaching the goal angle.
	This is handled by overshooting the goal angle by a predefined angle (usually 2 deg) and after a predefined time (usually 200 ms) it reverts to the actual goal angle.
	After this the servo is more or less exactly at the goal angle and the motor is turned off.
	This approach greatly reduces the current consumption in idle.
	Copyright(c) 2019 Vincent Groenhuis
	License: CC-BY-SA
*/

#ifndef SERVO_VALVE_H
#define SERVO_VALVE_H

class ServoValve {
public:
	// Constructor, all angles are in degrees.
	// neutralAngle must be specified and represents the angle at center position, usually with valve closed.
	// If lowAngle is not specified then it is computed as neutralAngle-45
	// If highAngle is not specified, then it is computed as neutralAngle+45
	ServoValve(int neutralAngle, int lowAngle=-1, int highAngle=-1, int minPulse=-1, int maxPulse=-1, int overshootAngle=2, int overshootPeriodMs=350, int logicalState = 2);

	// Returns true if not enough time has elapsed since last goal angle adjustment
	bool isMoving() const;

	// Returns goal angle (the 'final' setpointPosition)
	int getGoalAngle() const { return _goalAngle; }

	// Returns _setAngle, the current setpointPosition of the servo. It may be different from goalAngle to compensate for undershooting.
	int getSetAngle() const { return _setAngle; }

	// Returns pulse length as mapped from getSetAngle() to pulse using 0 deg = minPulse, 180 deg = maxPulse
	int getPulse() const;

	// state: 0=low, 1=high, 2=neutral
	void setLogicalState(int logicalState);

	// Sets goal angle (and logicalState to -1)
	void setGoalAngle(int goalAngle);

	// Sets goal angle relative to neutral angle
	void setGoalRelAngle(int goalRelAngle);

	// Returns and resets _changed
	bool changed();

	// may update _setAngle and _changed if necessary
	void work();

	// Prints a string representation of this servo to Serial (or to stdout in case of non-Arduino)
	// valveNr: a number which is also printed to better identify this valve
	void printState(int valveNr) const;
private:
	const int _neutralAngle;
	const int _lowAngle;
	const int _highAngle;
	const int _minPulse;
	const int _maxPulse;
	const int _overshootAngle; // usually 2 deg
	const int _overshootPeriodMs; // usually 350 ms

	int _logicalState; // 0=low, 1=high, 2=neutral, -1=custom angle
	int _goalAngle; // goal angle
	int _setAngle; // may be equal to _goalAngle, or off by +/-_overshootAngle
	int _lastMovementDir; // +1, -1 or 0

	unsigned long _lastAdjustTimeMs;
	bool _changed;
};



#endif