/*
    SPI1 is simpler than SPI0 bus as there is only one MCP device. This triggers the motor valves.
*/

#pragma once
#include <Arduino.h>
#include "PinDefinitions.h"
#include <SPI.h>
#include <MCP23S17.h>

class SPI1Manager {
public:
    SPI1Manager();
    bool begin();
    void writeValves(uint16_t valveStates);
    uint16_t getValveStates();

private:
    uint16_t _valveStates = 0;
    MCP23S17* _motorValves = nullptr;
    bool _initialized = false;
};