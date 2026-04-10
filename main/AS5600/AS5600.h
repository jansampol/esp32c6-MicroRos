#pragma once

#include <cstdint>
#include <cmath>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AS5600_LIB_VERSION "0.6.7-idf"

// default addresses
static constexpr uint8_t AS5600_DEFAULT_ADDRESS  = 0x36;
static constexpr uint8_t AS5600L_DEFAULT_ADDRESS = 0x40;
static constexpr int     AS5600_SW_DIRECTION_PIN = -1;

// setDirection
static constexpr uint8_t AS5600_CLOCK_WISE        = 0;
static constexpr uint8_t AS5600_COUNTERCLOCK_WISE = 1;

// conversion constants
static constexpr float AS5600_RAW_TO_DEGREES = 360.0f / 4096.0f;
static constexpr float AS5600_DEGREES_TO_RAW = 4096.0f / 360.0f;
static constexpr float AS5600_RAW_TO_RADIANS = (2.0f * static_cast<float>(M_PI)) / 4096.0f;
static constexpr float AS5600_RAW_TO_RPM     = 60.0f / 4096.0f;
static constexpr float AS5600_RAW_TO_RPS     = 1.0f / 4096.0f;

// getAngularSpeed modes
static constexpr uint8_t AS5600_MODE_DEGREES = 0;
static constexpr uint8_t AS5600_MODE_RADIANS = 1;
static constexpr uint8_t AS5600_MODE_RPM     = 2;
static constexpr uint8_t AS5600_MODE_RPS     = 3;

// error codes
static constexpr int AS5600_OK                = 0;
static constexpr int AS5600_ERROR_I2C_READ_0  = -100;
static constexpr int AS5600_ERROR_I2C_READ_1  = -101;
static constexpr int AS5600_ERROR_I2C_READ_2  = -102;
static constexpr int AS5600_ERROR_I2C_READ_3  = -103;
static constexpr int AS5600_ERROR_I2C_WRITE_0 = -200;
static constexpr int AS5600_ERROR_I2C_WRITE_1 = -201;

// configuration constants
static constexpr uint8_t AS5600_OUTMODE_ANALOG_100 = 0;
static constexpr uint8_t AS5600_OUTMODE_ANALOG_90  = 1;
static constexpr uint8_t AS5600_OUTMODE_PWM        = 2;

static constexpr uint8_t AS5600_POWERMODE_NOMINAL = 0;
static constexpr uint8_t AS5600_POWERMODE_LOW1    = 1;
static constexpr uint8_t AS5600_POWERMODE_LOW2    = 2;
static constexpr uint8_t AS5600_POWERMODE_LOW3    = 3;

static constexpr uint8_t AS5600_PWM_115 = 0;
static constexpr uint8_t AS5600_PWM_230 = 1;
static constexpr uint8_t AS5600_PWM_460 = 2;
static constexpr uint8_t AS5600_PWM_920 = 3;

static constexpr uint8_t AS5600_HYST_OFF  = 0;
static constexpr uint8_t AS5600_HYST_LSB1 = 1;
static constexpr uint8_t AS5600_HYST_LSB2 = 2;
static constexpr uint8_t AS5600_HYST_LSB3 = 3;

static constexpr uint8_t AS5600_SLOW_FILT_16X = 0;
static constexpr uint8_t AS5600_SLOW_FILT_8X  = 1;
static constexpr uint8_t AS5600_SLOW_FILT_4X  = 2;
static constexpr uint8_t AS5600_SLOW_FILT_2X  = 3;

static constexpr uint8_t AS5600_FAST_FILT_NONE  = 0;
static constexpr uint8_t AS5600_FAST_FILT_LSB6  = 1;
static constexpr uint8_t AS5600_FAST_FILT_LSB7  = 2;
static constexpr uint8_t AS5600_FAST_FILT_LSB9  = 3;
static constexpr uint8_t AS5600_FAST_FILT_LSB18 = 4;
static constexpr uint8_t AS5600_FAST_FILT_LSB21 = 5;
static constexpr uint8_t AS5600_FAST_FILT_LSB24 = 6;
static constexpr uint8_t AS5600_FAST_FILT_LSB10 = 7;

static constexpr uint8_t AS5600_WATCHDOG_OFF = 0;
static constexpr uint8_t AS5600_WATCHDOG_ON  = 1;

class AS5600
{
public:
    explicit AS5600(i2c_master_dev_handle_t i2c_dev);

    bool begin(gpio_num_t directionPin = static_cast<gpio_num_t>(AS5600_SW_DIRECTION_PIN));
    virtual bool isConnected();

    uint8_t getAddress();

    void    setDirection(uint8_t direction = AS5600_CLOCK_WISE);
    uint8_t getDirection();

    uint8_t  getZMCO();
    bool     setZPosition(uint16_t value);
    uint16_t getZPosition();

    bool     setMPosition(uint16_t value);
    uint16_t getMPosition();

    bool     setMaxAngle(uint16_t value);
    uint16_t getMaxAngle();

    bool     setConfiguration(uint16_t value);
    uint16_t getConfiguration();

    bool    setPowerMode(uint8_t powerMode);
    uint8_t getPowerMode();

    bool    setHysteresis(uint8_t hysteresis);
    uint8_t getHysteresis();

    bool    setOutputMode(uint8_t outputMode);
    uint8_t getOutputMode();

    bool    setPWMFrequency(uint8_t pwmFreq);
    uint8_t getPWMFrequency();

    bool    setSlowFilter(uint8_t mask);
    uint8_t getSlowFilter();

    bool    setFastFilter(uint8_t mask);
    uint8_t getFastFilter();

    bool    setWatchDog(uint8_t mask);
    uint8_t getWatchDog();

    void resetPOR();

    uint16_t rawAngle();
    uint16_t readAngle();

    bool  setOffset(float degrees);
    float getOffset();
    bool  increaseOffset(float degrees);

    uint8_t  readStatus();
    uint8_t  readAGC();
    uint16_t readMagnitude();

    bool magnetDetected();
    bool magnetTooStrong();
    bool magnetTooWeak();

    float   getAngularSpeed(uint8_t mode = AS5600_MODE_DEGREES, bool update = true);
    int32_t getCumulativePosition(bool update = true);
    int32_t getRevolutions();
    int32_t resetPosition(int32_t position = 0);
    int32_t resetCumulativePosition(int32_t position = 0);

    int lastError();

protected:
    virtual uint8_t  readReg(uint8_t reg);
    virtual uint16_t readReg2(uint8_t reg);
    virtual uint8_t  writeReg(uint8_t reg, uint8_t value);
    virtual uint8_t  writeReg2(uint8_t reg, uint16_t value);

    esp_err_t write_then_read(uint8_t reg, uint8_t* data, size_t len);

    uint8_t _address = AS5600_DEFAULT_ADDRESS;
    int     _directionPin = AS5600_SW_DIRECTION_PIN;
    uint8_t _direction = AS5600_CLOCK_WISE;
    int     _error = AS5600_OK;

    i2c_master_dev_handle_t _i2c_dev = nullptr;

    uint64_t _lastMeasurement = 0;
    int16_t  _lastAngle = 0;
    int16_t  _lastReadAngle = 0;

    uint16_t _offset = 0;

    int32_t _position = 0;
    int16_t _lastPosition = 0;
};

class AS5600L : public AS5600
{
public:
    explicit AS5600L(i2c_master_dev_handle_t i2c_dev, uint8_t address = AS5600L_DEFAULT_ADDRESS);

    bool    setAddress(uint8_t address);
    bool    setI2CUPDT(uint8_t value);
    uint8_t getI2CUPDT();
};