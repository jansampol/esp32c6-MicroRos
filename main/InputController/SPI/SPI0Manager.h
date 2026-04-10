#pragma once

#include <cstdint>

#include "pinDefinitions.h"
#include "SystemParameters.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

// ============================================================
// Arduino-dependent libraries intentionally disabled for now
// ============================================================
// #include <Adafruit_ST7789.h>
// #include <Adafruit_GFX.h>
// #include <MCP3XXX.h>

#define NUM_SPI0_DEVICES 7

enum deviceNameSPI0 {
    NO_DEVICE = 0,
    UI_1_HBUTTONS = 1,
    UI_2_LEDS_AND_SWITCHES = 2,
    UI_3_VBUTTONS = 3,
    MAINVALVE = 4,
    ADC_POTMETERS = 5,
    SCREEN = 6,
    EXTERNAL_MCP_1 = 7,
};

class SPI0Manager {
public:
    SPI0Manager();

    bool begin();
    void debugCSPins();

    void selectDevice(deviceNameSPI0 device);
    void deselectDevice();

    void setMainValve(bool on);
    void setScreenRES(bool on);
    void setScreenBLK(bool on);
    void writeMainValve(uint16_t state);

    uint16_t readHorizontalButtons();
    uint16_t readVerticalButtons();
    uint8_t readSwitches();

    void writeGreen(bool on);
    void writeRed1(bool on);
    void writeRed2(bool on);
    void writeRed3(bool on);
    void writeYellow(bool on);
    void writeBlue(bool on);
    void writeLeds(uint8_t ledStates);

    uint16_t readPotmeters(bool id);

    uint16_t readExternalDevice(deviceNameSPI0 device);
    void writeExternalDevice(deviceNameSPI0 device, uint16_t data);

    void debugReadMcpRegisters(deviceNameSPI0 device);
    void debugWriteAndReadLeds(uint8_t value);

    // Screen support intentionally disabled for now
    // Adafruit_ST7789& getScreen() { return _screen; }

private:
    esp_err_t initDemuxPins();

    esp_err_t mcpWrite8(deviceNameSPI0 device, uint8_t reg, uint8_t value);
    esp_err_t mcpWrite16(deviceNameSPI0 device, uint8_t regA, uint16_t value);
    esp_err_t mcpRead8(deviceNameSPI0 device, uint8_t reg, uint8_t& value);
    esp_err_t mcpRead16(deviceNameSPI0 device, uint8_t regA, uint16_t& value);

    esp_err_t mcpInitMCP23S17(deviceNameSPI0 device,
                              uint16_t iodir,
                              uint16_t gppu,
                              uint16_t ipol,
                              uint16_t initial_gpio);

    uint8_t invertByte(uint8_t b);
    uint16_t swapLastHorizontalButtonPairs(uint16_t bits);

    void setLedBit(uint8_t bitIndex, bool on);
    

private:
    uint8_t _ledStates = 0;
    uint16_t _mainValveState = 0;
    deviceNameSPI0 _selectedDevice0 = NO_DEVICE;

    bool _initialized = false;
    spi_device_handle_t _spi_dev = nullptr;
};