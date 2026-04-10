#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#define TCA9548_LIB_VERSION "0.3.1-idf"

// error codes
static constexpr int TCA9548_OK            = 0;
static constexpr int TCA9548_ERROR_I2C     = -10;
static constexpr int TCA9548_ERROR_CHANNEL = -20;

class TCA9548
{
public:
    // deviceAddress = 0x70 .. 0x77
    TCA9548(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev);

    bool    begin(uint8_t mask = 0x00);  // default no channels enabled
    bool    isConnected();               // find multiplexer on I2C bus
    bool    isConnected(uint8_t address);                     // generic probe
    bool    isConnected(uint8_t address, uint8_t channel);   // probe through selected channel

    uint8_t find(uint8_t address);  // returns a channel mask

    uint8_t channelCount();
    bool    enableChannel(uint8_t channel);
    bool    disableChannel(uint8_t channel);
    bool    selectChannel(uint8_t channel);
    bool    isEnabled(uint8_t channel);
    bool    disableAllChannels();

    bool    setChannelMask(uint8_t mask);
    virtual uint8_t getChannelMask();

    void    setResetPin(gpio_num_t resetPin);
    void    reset();

    void    setForced(bool forced = false);
    bool    getForced();

    int     getError();

protected:
    bool    probeAddress(uint8_t address);
    esp_err_t readOneByte(uint8_t* value);

    uint8_t _address;
    i2c_master_dev_handle_t _i2c_dev;
    uint8_t _mask;
    int     _resetPin;   // AS5600-style sentinel: -1 means not set
    bool    _forced;
    int     _error;
    uint8_t _channels;
};

class PCA9548 : public TCA9548
{
public:
    PCA9548(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev);
};

class PCA9546 : public TCA9548
{
public:
    PCA9546(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev);
    uint8_t getChannelMask() override;
};

class PCA9545 : public TCA9548
{
public:
    PCA9545(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev);
    uint8_t getChannelMask() override;
    uint8_t getInterruptMask();
};

class PCA9543 : public TCA9548
{
public:
    PCA9543(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev);
    uint8_t getChannelMask() override;
    uint8_t getInterruptMask();
};