#include "InputController/I2C/I2CManager.h"

I2CManager::I2CManager()
{
}

I2CManager::~I2CManager()
{
    _ferrisWheelSensors.clear();
    _pressureSensorMultiplexer.reset();
    _externalMultiplexer1.reset();

    if (_as5600Dev) {
        i2c_master_bus_rm_device(_as5600Dev);
        _as5600Dev = nullptr;
    }
    if (_pressureSensorDev) {
        i2c_master_bus_rm_device(_pressureSensorDev);
        _pressureSensorDev = nullptr;
    }
    if (_externalMux1Dev) {
        i2c_master_bus_rm_device(_externalMux1Dev);
        _externalMux1Dev = nullptr;
    }
    if (_internalMuxDev) {
        i2c_master_bus_rm_device(_internalMuxDev);
        _internalMuxDev = nullptr;
    }
    if (_busHandle) {
        i2c_del_master_bus(_busHandle);
        _busHandle = nullptr;
    }
}

bool I2CManager::begin(uint8_t numberOfFerrisWheels, uint8_t otherSensors)
{
    ESP_LOGI(TAG, "Starting I2C Manager with %u Ferris wheels and %u other sensors...",
             numberOfFerrisWheels, otherSensors);

    if (numberOfFerrisWheels > MAX_FERRIS_WHEELS) {
        ESP_LOGE(TAG, "Invalid number of Ferris wheels: %u", numberOfFerrisWheels);
        return false;
    }

    _numFerrisWheels = numberOfFerrisWheels;

    if (!initBus()) {
        ESP_LOGE(TAG, "I2C bus failed to start");
        return false;
    }

    if (!initDevices()) {
        ESP_LOGE(TAG, "I2C device initialization failed");
        return false;
    }

    if (!_pressureSensorMultiplexer || !_pressureSensorMultiplexer->begin()) {
        ESP_LOGE(TAG, "Pressure sensor multiplexer failed to initialize");
        return false;
    }

    if (!_externalMultiplexer1 || !_externalMultiplexer1->begin()) {
        ESP_LOGE(TAG, "External multiplexer 1 failed to initialize");
        return false;
    }

    ESP_LOGI(TAG, "I2C multiplexers initialized. Initializing Ferris wheels...");

    bool ferrisResult = initFerrisWheels(numberOfFerrisWheels);
    if (!ferrisResult) {
        ESP_LOGE(TAG, "Failed to initialize Ferris wheels");
        return false;
    }

    ESP_LOGI(TAG, "Ferris wheels successfully initialized");

    for (uint8_t i = 0; i < _numFerrisWheels; i++) {
        _rawPosition[i] = _ferrisWheelSensors[i]->getCumulativePosition();
        _zeroOffset[i] = _rawPosition[i];
    }

    return true;
}

bool I2CManager::initBus()
{
    if (_busHandle != nullptr) {
        return true;
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_PIN);
    bus_config.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_PIN);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &_busHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2C bus started");
    return true;
}

bool I2CManager::initDevices()
{
    if (_busHandle == nullptr) {
        return false;
    }

    i2c_device_config_t mux_cfg = {};
    mux_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mux_cfg.scl_speed_hz = 400000;

    mux_cfg.device_address = TCA_INTERNAL_ADDR;
    if (_internalMuxDev == nullptr) {
        esp_err_t err = i2c_master_bus_add_device(_busHandle, &mux_cfg, &_internalMuxDev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed adding internal mux device: %s", esp_err_to_name(err));
            return false;
        }
    }

    mux_cfg.device_address = TCA_EXTERNAL1_ADDR;
    if (_externalMux1Dev == nullptr) {
        esp_err_t err = i2c_master_bus_add_device(_busHandle, &mux_cfg, &_externalMux1Dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed adding external mux 1 device: %s", esp_err_to_name(err));
            return false;
        }
    }

    i2c_device_config_t pressure_cfg = {};
    pressure_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    pressure_cfg.device_address = PRESSURE_SENSOR_ADDR;
    pressure_cfg.scl_speed_hz = 400000;

    if (_pressureSensorDev == nullptr) {
        esp_err_t err = i2c_master_bus_add_device(_busHandle, &pressure_cfg, &_pressureSensorDev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed adding pressure sensor device: %s", esp_err_to_name(err));
            return false;
        }
    }

    i2c_device_config_t as5600_cfg = {};
    as5600_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    as5600_cfg.device_address = AS5600_DEFAULT_ADDRESS;
    as5600_cfg.scl_speed_hz = 400000;

    if (_as5600Dev == nullptr) {
        esp_err_t err = i2c_master_bus_add_device(_busHandle, &as5600_cfg, &_as5600Dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed adding AS5600 device handle: %s", esp_err_to_name(err));
            return false;
        }
    }

    if (!_pressureSensorMultiplexer) {
        _pressureSensorMultiplexer = std::make_unique<TCA9548>(TCA_INTERNAL_ADDR, _internalMuxDev);
    }

    if (!_externalMultiplexer1) {
        _externalMultiplexer1 = std::make_unique<TCA9548>(TCA_EXTERNAL1_ADDR, _externalMux1Dev);
    }

    return true;
}

bool I2CManager::initFerrisWheels(uint8_t numberOfFerrisWheels)
{
    ESP_LOGI(TAG, "Initializing Ferris wheel sensors...");

    if (numberOfFerrisWheels > MAX_FERRIS_WHEELS) {
        ESP_LOGE(TAG, "Number of Ferris wheels exceeds maximum");
        return false;
    }

    _numFerrisWheels = numberOfFerrisWheels;

    _angleDeg.assign(_numFerrisWheels, 0.0f);
    _ferrisWheelSensors.clear();
    _ferrisWheelSensors.reserve(_numFerrisWheels);

    for (uint8_t i = 0; i < numberOfFerrisWheels; i++) {
        if (i < 8) {
            if (!_externalMultiplexer1->selectChannel(_wheelIdMapping[i])) {
                ESP_LOGE(TAG, "Failed to select external mux channel for wheel %u", i);
                return false;
            }
        } else {
            if (!_pressureSensorMultiplexer->selectChannel(_wheelIdMapping[i])) {
                ESP_LOGE(TAG, "Failed to select internal mux channel for wheel %u", i);
                return false;
            }
        }

        auto sensor = std::make_unique<AS5600>(_as5600Dev);

        bool thisFerrisResult = sensor->begin();
        ESP_LOGI(TAG, "Ferris wheel sensor %u: %d", i, thisFerrisResult ? 1 : 0);

        if (!thisFerrisResult) {
            ESP_LOGE(TAG, "Ferris wheel sensor %u failed to initialize", i);
            return false;
        }

        _ferrisWheelSensors.push_back(std::move(sensor));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return true;
}

float I2CManager::readFerrisWheelAngle(uint8_t wheelId)
{
    if (wheelId >= _numFerrisWheels) {
        ESP_LOGE(TAG, "Invalid Ferris wheel ID: %u", wheelId);
        return NAN;
    }

    if (wheelId < 8) {
        _externalMultiplexer1->selectChannel(_wheelIdMapping[wheelId]);
    } else {
        _pressureSensorMultiplexer->selectChannel(_wheelIdMapping[wheelId]);
    }

    _rawPosition[wheelId] = _ferrisWheelSensors[wheelId]->getCumulativePosition();
    _angleDeg[wheelId] = (_rawPosition[wheelId] - _zeroOffset[wheelId]) * _scaleFactor[wheelId];

    return _angleDeg[wheelId];
}

std::vector<float> I2CManager::readAllFerrisWheelAngles()
{
    for (uint8_t i = 0; i < _numFerrisWheels; i++) {
        _angleDeg[i] = readFerrisWheelAngle(i);
    }
    return _angleDeg;
}

float I2CManager::readFerrisWheelRawValue(uint8_t wheelId)
{
    if (wheelId >= _numFerrisWheels) {
        ESP_LOGE(TAG, "Invalid Ferris wheel ID: %u", wheelId);
        return NAN;
    }

    if (wheelId < 8) {
        _externalMultiplexer1->selectChannel(_wheelIdMapping[wheelId]);
    } else {
        _pressureSensorMultiplexer->selectChannel(_wheelIdMapping[wheelId]);
    }

    _rawPosition[wheelId] = _ferrisWheelSensors[wheelId]->getCumulativePosition();
    return static_cast<float>(_rawPosition[wheelId] - _zeroOffset[wheelId]);
}

std::vector<float> I2CManager::readAllFerrisWheelRawValues()
{
    std::vector<float> rawValues(_numFerrisWheels, 0.0f);
    for (uint8_t i = 0; i < _numFerrisWheels; i++) {
        rawValues[i] = readFerrisWheelRawValue(i);
    }
    return rawValues;
}

bool I2CManager::writePressureRegister(uint8_t reg, const uint8_t* data, size_t len)
{
    if (_pressureSensorDev == nullptr) {
        return false;
    }

    std::vector<uint8_t> payload;
    payload.reserve(len + 1);
    payload.push_back(reg);
    for (size_t i = 0; i < len; i++) {
        payload.push_back(data[i]);
    }

    esp_err_t err = i2c_master_transmit(_pressureSensorDev, payload.data(), payload.size(), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Pressure sensor write failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool I2CManager::readPressureRegister(uint8_t reg, uint8_t* data, size_t len)
{
    if (_pressureSensorDev == nullptr) {
        return false;
    }

    esp_err_t err = i2c_master_transmit_receive(_pressureSensorDev, &reg, 1, data, len, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Pressure sensor read failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

float I2CManager::readPressureSensor(uint8_t sensorId)
{
    if (sensorId >= _numOfPressureSensors) {
        ESP_LOGE(TAG, "Invalid pressure sensor ID: %u", sensorId);
        return NAN;
    }

    if (!_pressureSensorMultiplexer->selectChannel(sensorId)) {
        ESP_LOGE(TAG, "Failed to select pressure sensor channel: %u", sensorId);
        return NAN;
    }

    uint8_t status = 0;
    bool statusOk = false;

    if (readPressureRegister(0x02, &status, 1)) {
        statusOk = (status & 0x01) != 0;
    }

    if (!statusOk) {
        startPressureMeasurement();
        vTaskDelay(pdMS_TO_TICKS(10));

        if (readPressureRegister(0x02, &status, 1)) {
            statusOk = (status & 0x01) != 0;
        } else {
            return NAN;
        }
    }

    if (!statusOk) {
        return NAN;
    }

    uint8_t rawBytes[3] = {0, 0, 0};
    if (!readPressureRegister(0x06, rawBytes, 3)) {
        return NAN;
    }

    uint32_t rawPressure = 0;
    for (int i = 0; i < 3; i++) {
        rawPressure = (rawPressure << 8) | rawBytes[i];
    }

    startPressureMeasurement();

    float pressureNormalized = (rawPressure < 8388608U)
        ? static_cast<float>(rawPressure) / 8388608.0f
        : (static_cast<float>(rawPressure) - 16777216.0f) / 8388608.0f;

    float pressureBar = (pressureNormalized + 0.731f) * 10.0f;
    return pressureBar;
}

void I2CManager::startPressureMeasurement()
{
    uint8_t cmd = 0x0A;
    if (!writePressureRegister(0x30, &cmd, 1)) {
        ESP_LOGE(TAG, "Failed to start pressure measurement");
    }
}