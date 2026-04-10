#include "AS5600.h"

// configuration registers
static constexpr uint8_t AS5600_ZMCO = 0x00;
static constexpr uint8_t AS5600_ZPOS = 0x01;
static constexpr uint8_t AS5600_MPOS = 0x03;
static constexpr uint8_t AS5600_MANG = 0x05;
static constexpr uint8_t AS5600_CONF = 0x07;

// configuration masks
static constexpr uint8_t AS5600_CONF_POWER_MODE    = 0x03;
static constexpr uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
static constexpr uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
static constexpr uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
static constexpr uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
static constexpr uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
static constexpr uint8_t AS5600_CONF_WATCH_DOG     = 0x20;

// output/status registers
static constexpr uint8_t AS5600_STATUS    = 0x0B;
static constexpr uint8_t AS5600_RAW_ANGLE = 0x0C;
static constexpr uint8_t AS5600_ANGLE     = 0x0E;
static constexpr uint8_t AS5600_I2CADDR   = 0x20;
static constexpr uint8_t AS5600_I2CUPDT   = 0x21;
static constexpr uint8_t AS5600_AGC       = 0x1A;
static constexpr uint8_t AS5600_MAGNITUDE = 0x1B;
static constexpr uint8_t AS5600_BURN      = 0xFF;

// status bits
static constexpr uint8_t AS5600_MAGNET_HIGH   = 0x08;
static constexpr uint8_t AS5600_MAGNET_LOW    = 0x10;
static constexpr uint8_t AS5600_MAGNET_DETECT = 0x20;

AS5600::AS5600(i2c_master_dev_handle_t i2c_dev)
    : _i2c_dev(i2c_dev)
{
}

bool AS5600::begin(gpio_num_t directionPin)
{
    _directionPin = static_cast<int>(directionPin);

    if (_directionPin != AS5600_SW_DIRECTION_PIN)
    {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << _directionPin);
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
    }

    setDirection(AS5600_CLOCK_WISE);
    return isConnected();
}

bool AS5600::isConnected()
{
    uint8_t dummy = 0;
    esp_err_t err = write_then_read(AS5600_STATUS, &dummy, 1);
    return (err == ESP_OK);
}

uint8_t AS5600::getAddress()
{
    return _address;
}

void AS5600::setDirection(uint8_t direction)
{
    _direction = direction;
    if (_directionPin != AS5600_SW_DIRECTION_PIN)
    {
        gpio_set_level(static_cast<gpio_num_t>(_directionPin), _direction ? 1 : 0);
    }
}

uint8_t AS5600::getDirection()
{
    if (_directionPin != AS5600_SW_DIRECTION_PIN)
    {
        _direction = static_cast<uint8_t>(
            gpio_get_level(static_cast<gpio_num_t>(_directionPin)) ? 1 : 0
        );
    }
    return _direction;
}

uint8_t AS5600::getZMCO()
{
    return readReg(AS5600_ZMCO);
}

bool AS5600::setZPosition(uint16_t value)
{
    if (value > 0x0FFF) return false;
    writeReg2(AS5600_ZPOS, value);
    return true;
}

uint16_t AS5600::getZPosition()
{
    return readReg2(AS5600_ZPOS) & 0x0FFF;
}

bool AS5600::setMPosition(uint16_t value)
{
    if (value > 0x0FFF) return false;
    writeReg2(AS5600_MPOS, value);
    return true;
}

uint16_t AS5600::getMPosition()
{
    return readReg2(AS5600_MPOS) & 0x0FFF;
}

bool AS5600::setMaxAngle(uint16_t value)
{
    if (value > 0x0FFF) return false;
    writeReg2(AS5600_MANG, value);
    return true;
}

uint16_t AS5600::getMaxAngle()
{
    return readReg2(AS5600_MANG) & 0x0FFF;
}

bool AS5600::setConfiguration(uint16_t value)
{
    if (value > 0x3FFF) return false;
    writeReg2(AS5600_CONF, value);
    return true;
}

uint16_t AS5600::getConfiguration()
{
    return readReg2(AS5600_CONF) & 0x3FFF;
}

bool AS5600::setPowerMode(uint8_t powerMode)
{
    if (powerMode > 3) return false;
    uint8_t value = readReg(AS5600_CONF + 1);
    value &= ~AS5600_CONF_POWER_MODE;
    value |= powerMode;
    writeReg(AS5600_CONF + 1, value);
    return true;
}

uint8_t AS5600::getPowerMode()
{
    return readReg(AS5600_CONF + 1) & 0x03;
}

bool AS5600::setHysteresis(uint8_t hysteresis)
{
    if (hysteresis > 3) return false;
    uint8_t value = readReg(AS5600_CONF + 1);
    value &= ~AS5600_CONF_HYSTERESIS;
    value |= (hysteresis << 2);
    writeReg(AS5600_CONF + 1, value);
    return true;
}

uint8_t AS5600::getHysteresis()
{
    return (readReg(AS5600_CONF + 1) >> 2) & 0x03;
}

bool AS5600::setOutputMode(uint8_t outputMode)
{
    if (outputMode > 2) return false;
    uint8_t value = readReg(AS5600_CONF + 1);
    value &= ~AS5600_CONF_OUTPUT_MODE;
    value |= (outputMode << 4);
    writeReg(AS5600_CONF + 1, value);
    return true;
}

uint8_t AS5600::getOutputMode()
{
    return (readReg(AS5600_CONF + 1) >> 4) & 0x03;
}

bool AS5600::setPWMFrequency(uint8_t pwmFreq)
{
    if (pwmFreq > 3) return false;
    uint8_t value = readReg(AS5600_CONF + 1);
    value &= ~AS5600_CONF_PWM_FREQUENCY;
    value |= (pwmFreq << 6);
    writeReg(AS5600_CONF + 1, value);
    return true;
}

uint8_t AS5600::getPWMFrequency()
{
    return (readReg(AS5600_CONF + 1) >> 6) & 0x03;
}

bool AS5600::setSlowFilter(uint8_t mask)
{
    if (mask > 3) return false;
    uint8_t value = readReg(AS5600_CONF);
    value &= ~AS5600_CONF_SLOW_FILTER;
    value |= mask;
    writeReg(AS5600_CONF, value);
    return true;
}

uint8_t AS5600::getSlowFilter()
{
    return readReg(AS5600_CONF) & 0x03;
}

bool AS5600::setFastFilter(uint8_t mask)
{
    if (mask > 7) return false;
    uint8_t value = readReg(AS5600_CONF);
    value &= ~AS5600_CONF_FAST_FILTER;
    value |= (mask << 2);
    writeReg(AS5600_CONF, value);
    return true;
}

uint8_t AS5600::getFastFilter()
{
    return (readReg(AS5600_CONF) >> 2) & 0x07;
}

bool AS5600::setWatchDog(uint8_t mask)
{
    if (mask > 1) return false;
    uint8_t value = readReg(AS5600_CONF);
    value &= ~AS5600_CONF_WATCH_DOG;
    value |= (mask << 5);
    writeReg(AS5600_CONF, value);
    return true;
}

uint8_t AS5600::getWatchDog()
{
    return (readReg(AS5600_CONF) >> 5) & 0x01;
}

uint16_t AS5600::rawAngle()
{
    int16_t value = static_cast<int16_t>(readReg2(AS5600_RAW_ANGLE));
    if (_offset > 0) value += _offset;
    value &= 0x0FFF;

    if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
        (_direction == AS5600_COUNTERCLOCK_WISE))
    {
        value = (4096 - value) & 0x0FFF;
    }
    return static_cast<uint16_t>(value);
}

uint16_t AS5600::readAngle()
{
    uint16_t value = readReg2(AS5600_ANGLE);
    if (_error != AS5600_OK)
    {
        return static_cast<uint16_t>(_lastReadAngle);
    }

    if (_offset > 0) value += _offset;
    value &= 0x0FFF;

    if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
        (_direction == AS5600_COUNTERCLOCK_WISE))
    {
        value = (4096 - value) & 0x0FFF;
    }

    _lastReadAngle = static_cast<int16_t>(value);
    return value;
}

bool AS5600::setOffset(float degrees)
{
    if (std::fabs(degrees) > 36000.0f) return false;

    bool neg = (degrees < 0.0f);
    if (neg) degrees = -degrees;

    uint16_t offset = static_cast<uint16_t>(std::lround(degrees * AS5600_DEGREES_TO_RAW));
    offset &= 0x0FFF;

    if (neg) offset = (4096 - offset) & 0x0FFF;
    _offset = offset;
    return true;
}

float AS5600::getOffset()
{
    return _offset * AS5600_RAW_TO_DEGREES;
}

bool AS5600::increaseOffset(float degrees)
{
    return setOffset((_offset * AS5600_RAW_TO_DEGREES) + degrees);
}

uint8_t AS5600::readStatus()
{
    return readReg(AS5600_STATUS);
}

uint8_t AS5600::readAGC()
{
    return readReg(AS5600_AGC);
}

uint16_t AS5600::readMagnitude()
{
    return readReg2(AS5600_MAGNITUDE) & 0x0FFF;
}

bool AS5600::magnetDetected()
{
    return (readStatus() & AS5600_MAGNET_DETECT) != 0;
}

bool AS5600::magnetTooStrong()
{
    return (readStatus() & AS5600_MAGNET_HIGH) != 0;
}

bool AS5600::magnetTooWeak()
{
    return (readStatus() & AS5600_MAGNET_LOW) != 0;
}

void AS5600::resetPOR()
{
    writeReg(AS5600_BURN, 0x01);
    writeReg(AS5600_BURN, 0x11);
    writeReg(AS5600_BURN, 0x10);
    vTaskDelay(pdMS_TO_TICKS(5));
}

float AS5600::getAngularSpeed(uint8_t mode, bool update)
{
    if (update)
    {
        _lastReadAngle = static_cast<int16_t>(readAngle());
        if (_error != AS5600_OK)
        {
            return NAN;
        }
    }

    uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    int angle = _lastReadAngle;
    uint64_t deltaT = now - _lastMeasurement;
    int deltaA = angle - _lastAngle;

    if (deltaT == 0) return NAN;

    if (deltaA > 2048) deltaA -= 4096;
    else if (deltaA < -2048) deltaA += 4096;

    float speed = (deltaA * 1e6f) / static_cast<float>(deltaT);

    _lastMeasurement = now;
    _lastAngle = static_cast<int16_t>(angle);

    if (mode == AS5600_MODE_RADIANS) return speed * AS5600_RAW_TO_RADIANS;
    if (mode == AS5600_MODE_RPM)     return speed * AS5600_RAW_TO_RPM;
    if (mode == AS5600_MODE_RPS)     return speed * AS5600_RAW_TO_RPS;
    return speed * AS5600_RAW_TO_DEGREES;
}

int32_t AS5600::getCumulativePosition(bool update)
{
    if (update)
    {
        _lastReadAngle = static_cast<int16_t>(readAngle());
        if (_error != AS5600_OK)
        {
            return _position;
        }
    }

    int16_t value = _lastReadAngle;

    if ((_lastPosition > 2048) && (value < (_lastPosition - 2048)))
    {
        _position = _position + 4096 - _lastPosition + value;
    }
    else if ((value > 2048) && (_lastPosition < (value - 2048)))
    {
        _position = _position - 4096 - _lastPosition + value;
    }
    else
    {
        _position = _position - _lastPosition + value;
    }

    _lastPosition = value;
    return _position;
}

int32_t AS5600::getRevolutions()
{
    int32_t p = _position >> 12;
    if (p < 0) p++;
    return p;
}

int32_t AS5600::resetPosition(int32_t position)
{
    int32_t old = _position;
    _position = position;
    return old;
}

int32_t AS5600::resetCumulativePosition(int32_t position)
{
    _lastPosition = static_cast<int16_t>(readAngle());
    int32_t old = _position;
    _position = position;
    return old;
}

int AS5600::lastError()
{
    int value = _error;
    _error = AS5600_OK;
    return value;
}

esp_err_t AS5600::write_then_read(uint8_t reg, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(_i2c_dev, &reg, 1, data, len, -1);
}

uint8_t AS5600::readReg(uint8_t reg)
{
    _error = AS5600_OK;

    uint8_t data = 0;
    esp_err_t err = write_then_read(reg, &data, 1);
    if (err != ESP_OK)
    {
        _error = AS5600_ERROR_I2C_READ_0;
        return 0;
    }

    return data;
}

uint16_t AS5600::readReg2(uint8_t reg)
{
    _error = AS5600_OK;

    uint8_t data[2] = {0, 0};
    esp_err_t err = write_then_read(reg, data, 2);
    if (err != ESP_OK)
    {
        _error = AS5600_ERROR_I2C_READ_2;
        return 0;
    }

    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

uint8_t AS5600::writeReg(uint8_t reg, uint8_t value)
{
    _error = AS5600_OK;

    uint8_t payload[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(_i2c_dev, payload, sizeof(payload), -1);
    if (err != ESP_OK)
    {
        _error = AS5600_ERROR_I2C_WRITE_0;
    }

    return static_cast<uint8_t>(_error);
}

uint8_t AS5600::writeReg2(uint8_t reg, uint16_t value)
{
    _error = AS5600_OK;

    uint8_t payload[3] = {
        reg,
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };

    esp_err_t err = i2c_master_transmit(_i2c_dev, payload, sizeof(payload), -1);
    if (err != ESP_OK)
    {
        _error = AS5600_ERROR_I2C_WRITE_0;
    }

    return static_cast<uint8_t>(_error);
}

AS5600L::AS5600L(i2c_master_dev_handle_t i2c_dev, uint8_t address)
    : AS5600(i2c_dev)
{
    _address = address;
}

bool AS5600L::setAddress(uint8_t address)
{
    if ((address < 8) || (address > 119)) return false;

    writeReg(AS5600_I2CADDR, address << 1);
    writeReg(AS5600_I2CUPDT, address << 1);
    _address = address;
    return true;
}

bool AS5600L::setI2CUPDT(uint8_t address)
{
    if ((address < 8) || (address > 119)) return false;
    writeReg(AS5600_I2CUPDT, address << 1);
    return true;
}

uint8_t AS5600L::getI2CUPDT()
{
    return (readReg(AS5600_I2CUPDT) >> 1);
}