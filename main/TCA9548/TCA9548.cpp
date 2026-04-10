#include "TCA9548.h"

TCA9548::TCA9548(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev)
{
    _address  = deviceAddress;
    _i2c_dev  = i2c_dev;
    _mask     = 0x00;
    _resetPin = -1;
    _forced   = false;
    _error    = TCA9548_OK;
    _channels = 8;
}

bool TCA9548::begin(uint8_t mask)
{
    if (!isConnected()) return false;
    return setChannelMask(mask);
}

bool TCA9548::isConnected()
{
    return probeAddress(_address);
}

bool TCA9548::isConnected(uint8_t address)
{
    return probeAddress(address);
}

bool TCA9548::isConnected(uint8_t address, uint8_t channel)
{
    if (!selectChannel(channel)) return false;
    return probeAddress(address);
}

uint8_t TCA9548::find(uint8_t address)
{
    uint8_t mask = 0x00;
    for (uint8_t ch = 0; ch < _channels; ch++)
    {
        selectChannel(ch);
        if (probeAddress(address)) mask |= (1U << ch);
    }
    return mask;
}

uint8_t TCA9548::channelCount()
{
    return _channels;
}

bool TCA9548::enableChannel(uint8_t channel)
{
    if (channel >= _channels)
    {
        _error = TCA9548_ERROR_CHANNEL;
        return false;
    }
    return setChannelMask(_mask | (0x01U << channel));
}

bool TCA9548::disableChannel(uint8_t channel)
{
    if (channel >= _channels)
    {
        _error = TCA9548_ERROR_CHANNEL;
        return false;
    }
    return setChannelMask(_mask & ~(0x01U << channel));
}

bool TCA9548::selectChannel(uint8_t channel)
{
    if (channel >= _channels)
    {
        _error = TCA9548_ERROR_CHANNEL;
        return false;
    }
    return setChannelMask(0x01U << channel);
}

bool TCA9548::isEnabled(uint8_t channel)
{
    if (channel >= _channels) return false;
    return (_mask & (0x01U << channel)) != 0;
}

bool TCA9548::disableAllChannels()
{
    return setChannelMask(0x00);
}

bool TCA9548::setChannelMask(uint8_t mask)
{
    if ((_mask == mask) && (!_forced)) return true;

    _mask = mask;
    esp_err_t err = i2c_master_transmit(_i2c_dev, &_mask, 1, -1);
    _error = (err == ESP_OK) ? TCA9548_OK : TCA9548_ERROR_I2C;
    return (_error == TCA9548_OK);
}

uint8_t TCA9548::getChannelMask()
{
    if (_forced)
    {
        uint8_t value = 0;
        if (readOneByte(&value) == ESP_OK)
        {
            _mask = value;
        }
    }
    return _mask;
}

void TCA9548::setResetPin(gpio_num_t resetPin)
{
    _resetPin = static_cast<int>(resetPin);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << _resetPin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // datasheet behavior preserved: HIGH = normal operation
    gpio_set_level(static_cast<gpio_num_t>(_resetPin), 1);
}

void TCA9548::reset()
{
    if (_resetPin < 0) return;

    gpio_set_level(static_cast<gpio_num_t>(_resetPin), 0);
    esp_rom_delay_us(1);   // 1 us pulse
    gpio_set_level(static_cast<gpio_num_t>(_resetPin), 1);
}

void TCA9548::setForced(bool forced)
{
    _forced = forced;
}

bool TCA9548::getForced()
{
    return _forced;
}

int TCA9548::getError()
{
    int error = _error;
    _error = TCA9548_OK;
    return error;
}

bool TCA9548::probeAddress(uint8_t address)
{
    // This uses a temporary device config because ESP-IDF transactions operate on device handles.
    // In your project, if you probe often, it is better to centralize this in your I2C manager.

    i2c_master_bus_handle_t bus_handle = nullptr;
    esp_err_t err = i2c_master_get_bus_handle(I2C_NUM_0, &bus_handle);
    if (err != ESP_OK || bus_handle == nullptr)
    {
        _error = TCA9548_ERROR_I2C;
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = 100000;

    i2c_master_dev_handle_t temp_dev = nullptr;
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &temp_dev);
    if (err != ESP_OK)
    {
        _error = TCA9548_ERROR_I2C;
        return false;
    }

    uint8_t dummy = 0;
    err = i2c_master_receive(temp_dev, &dummy, 1, 10);

    i2c_master_bus_rm_device(temp_dev);

    _error = (err == ESP_OK) ? TCA9548_OK : TCA9548_ERROR_I2C;
    return err == ESP_OK;
}

esp_err_t TCA9548::readOneByte(uint8_t* value)
{
    return i2c_master_receive(_i2c_dev, value, 1, -1);
}


// PCA9548
PCA9548::PCA9548(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev)
    : TCA9548(deviceAddress, i2c_dev)
{
    _channels = 8;
}

// PCA9546
PCA9546::PCA9546(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev)
    : TCA9548(deviceAddress, i2c_dev)
{
    _channels = 4;
}

uint8_t PCA9546::getChannelMask()
{
    if (_forced)
    {
        uint8_t value = 0;
        if (readOneByte(&value) == ESP_OK)
        {
            _mask = value;
        }
    }
    _mask &= 0x0F;
    return _mask;
}

// PCA9545
PCA9545::PCA9545(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev)
    : TCA9548(deviceAddress, i2c_dev)
{
    _channels = 4;
}

uint8_t PCA9545::getChannelMask()
{
    if (_forced)
    {
        uint8_t value = 0;
        if (readOneByte(&value) == ESP_OK)
        {
            _mask = value;
        }
    }
    _mask &= 0x0F;
    return _mask;
}

uint8_t PCA9545::getInterruptMask()
{
    uint8_t value = 0;
    if (readOneByte(&value) != ESP_OK) return 0;
    return static_cast<uint8_t>(value >> 4);
}

// PCA9543
PCA9543::PCA9543(uint8_t deviceAddress, i2c_master_dev_handle_t i2c_dev)
    : TCA9548(deviceAddress, i2c_dev)
{
    _channels = 2;
}

uint8_t PCA9543::getChannelMask()
{
    if (_forced)
    {
        uint8_t value = 0;
        if (readOneByte(&value) == ESP_OK)
        {
            _mask = value;
        }
    }
    _mask &= 0x03;
    return _mask;
}

uint8_t PCA9543::getInterruptMask()
{
    uint8_t value = 0;
    if (readOneByte(&value) != ESP_OK) return 0;
    return static_cast<uint8_t>(value >> 4);
}