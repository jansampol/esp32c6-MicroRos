#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "../../pinDefinitions.h"

class SPI1Manager {
public:
    SPI1Manager();

    bool begin();
    void writeValves(uint16_t valveStates);
    uint16_t getValveStates();

private:
    esp_err_t mcpWrite8(uint8_t reg, uint8_t value);
    esp_err_t mcpWrite16(uint8_t regA, uint16_t value);
    esp_err_t mcpInit();

private:
    uint16_t _valveStates = 0;
    bool _initialized = false;
    spi_device_handle_t _spi_dev = nullptr;
};