#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <iomanip>
using namespace std;
// When testing the library, the millis(), micros() and map(...) functions must be defined
// by the testing framework.
extern unsigned long millis();
extern unsigned long micros();
extern long map(long x, long in_min, long in_max, long out_min, long out_max);
/* 
// example implementation for map(...):
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
#endif

#include "ServoValve.h"

int ServoValve::getPulse() const {
	return map(getSetAngle(), 0, 180, _minPulse, _maxPulse);
}

ServoValve::ServoValve(int neutralAngle, int lowAngle, int highAngle, int minPulse, int maxPulse, int overshootAngle, int overshootPeriodMs, int logicalState)
	: _neutralAngle(neutralAngle), _lowAngle(lowAngle==-1?neutralAngle-45:lowAngle), _highAngle(highAngle==-1?neutralAngle+45:highAngle),
	_minPulse(minPulse), _maxPulse(maxPulse), _overshootAngle(overshootAngle), _overshootPeriodMs(overshootPeriodMs), _logicalState(-1),  _lastMovementDir(0)
{
	setLogicalState(logicalState);
	_lastMovementDir = 0;
	_changed = true;
	work();
}

bool ServoValve::isMoving() const {
	return (millis() - _lastAdjustTimeMs) < (unsigned long)_overshootPeriodMs;
}

/*
	Sets logical state and updates _goalAngle, _lastMovementDir and resets _lastAdjustTimeMs to current time
	newState: 0=low, 1=high, 2=neutral
*/
void ServoValve::setLogicalState(int logicalState) {
	if (logicalState == _logicalState) {
		return;
	}
	_logicalState = logicalState;
	int goalAngle = (_logicalState == 0 ? _lowAngle : (_logicalState == 1 ? _highAngle : _neutralAngle));
	_lastMovementDir = (goalAngle > _goalAngle) ? 1 : ((goalAngle < _goalAngle) ? -1 : 0);
	_goalAngle = goalAngle;
	_lastAdjustTimeMs = millis();
	work();
}

void ServoValve::setGoalAngle(int goalAngle) {
	if (goalAngle == _goalAngle) {
		return;
	}
	_logicalState = -1;
	_lastMovementDir = (goalAngle > _goalAngle) ? 1 : ((goalAngle < _goalAngle) ? -1 : 0);
	_goalAngle = goalAngle;
	_lastAdjustTimeMs = millis();
	work();
}

void ServoValve::setGoalRelAngle(int goalRelAngle) {
	setGoalAngle(_neutralAngle + goalRelAngle);
}

void ServoValve::work() {
	bool angleChanged = false;
	int newAngle = _goalAngle;
	if (isMoving()) {
		newAngle += _lastMovementDir*_overshootAngle;
	}
	if (newAngle != _setAngle) {
		angleChanged = true;
		_setAngle = newAngle;
	}
	_changed |= angleChanged;
}

bool ServoValve::changed() {
	bool ch = _changed;
	_changed = false;
	return ch;
}


void ServoValve::printState(int valveNr) const {
#ifdef ARDUINO
	Serial.print("Valve ");
	Serial.print(valveNr);
	Serial.print(": state ");
	Serial.print(_logicalState);
	Serial.print(" goal=");
	Serial.print(_goalAngle);
	Serial.print(" cur=");
	Serial.print(_setAngle);
	Serial.print(" lastDir=");
	Serial.print(_lastMovementDir);
	Serial.println();
#else
	cout << "Valve " << valveNr << " state " << _logicalState << " goal=" << _goalAngle << " cur=" << _setAngle << " lastDir=" << _lastMovementDir << endl;
#endif
}
