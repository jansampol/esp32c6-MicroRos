#include "InputController/SPI/SPI1Manager.h"
#include <SPI.h>

SPI1Manager::SPI1Manager()
{
}

bool SPI1Manager::begin() {
    if (_initialized) {
        return true;
    }

    // Create MCP object only at runtime, not during static construction
    _motorValves = new MCP23S17(
        SPI1_CS_MOTOR,
        SPI1_MISO_PIN,
        SPI1_MOSI_PIN,
        SPI1_CLK_PIN,
        0x00
    );

    if (!_motorValves->begin()) {
        Serial.println("---- Motor MCP23S17 failed to initialize! ----");
        return false;
    }

    _motorValves->pinMode16(0x0000); // all outputs
    _initialized = true;
    return true;
}

void SPI1Manager::writeValves(uint16_t valveStates) {
    _valveStates = valveStates;

    if (!_initialized || _motorValves == nullptr) {
        return;
    }

    _motorValves->write16(valveStates);
}

uint16_t SPI1Manager::getValveStates() {
    return _valveStates;
}