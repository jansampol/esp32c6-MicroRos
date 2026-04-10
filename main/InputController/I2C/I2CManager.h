#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "../../pinDefinitions.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../TCA9548/TCA9548.h"
#include "../../AS5600/AS5600.h"

#define TCA_INTERNAL_ADDR   0x70
#define TCA_EXTERNAL1_ADDR  0x71
// #define TCA_EXTERNAL2_ADDR  0x72

#define PRESSURE_SENSOR_ADDR 0x6D

#define MAX_FERRIS_WHEELS 14
#define EXT_I2C_DEVICES 4

class I2CManager
{
public:
    I2CManager();
    ~I2CManager();

    bool begin(uint8_t numberOfFerrisWheels, uint8_t otherSensors = 0);
    bool initFerrisWheels(uint8_t numberOfFerrisWheels);

    float readFerrisWheelAngle(uint8_t wheelId);
    std::vector<float> readAllFerrisWheelAngles();

    float readFerrisWheelRawValue(uint8_t wheelId);
    std::vector<float> readAllFerrisWheelRawValues();

    float readPressureSensor(uint8_t sensorId);
    void  startPressureMeasurement();

    uint8_t getNumOfFerrisWheels() const { return _numFerrisWheels; }
    uint8_t getNumOfPressureSensors() const { return _numOfPressureSensors; }

private:
    bool initBus();
    bool initDevices();

    bool writePressureRegister(uint8_t reg, const uint8_t* data, size_t len);
    bool readPressureRegister(uint8_t reg, uint8_t* data, size_t len);

private:
    static constexpr const char* TAG = "I2CManager";

    uint8_t _numFerrisWheels = 0;
    const uint8_t _numOfPressureSensors = 2;

    const uint8_t _wheelIdMapping[14] = {
        0, 1, 2, 3, 4, 5, 6, 7,
        2, 3, 4, 5, 6, 7
    };

    // ESP-IDF I2C bus + devices
    i2c_master_bus_handle_t _busHandle = nullptr;

    i2c_master_dev_handle_t _internalMuxDev = nullptr;
    i2c_master_dev_handle_t _externalMux1Dev = nullptr;
    i2c_master_dev_handle_t _pressureSensorDev = nullptr;
    i2c_master_dev_handle_t _as5600Dev = nullptr;

    // multiplexers
    std::unique_ptr<TCA9548> _pressureSensorMultiplexer;
    std::unique_ptr<TCA9548> _externalMultiplexer1;
    // std::unique_ptr<TCA9548> _externalMultiplexer2;

    // ferris wheel sensors
    std::vector<std::unique_ptr<AS5600>> _ferrisWheelSensors;

    int32_t _rawPosition[16] = {0};
    std::vector<float> _angleDeg;

    int32_t _zeroOffset[16] = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    };

    const float _scaleFactor[16] = {
        (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f),
        (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f),
        (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f),
        (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f), (360.0f / 4096.0f)
    };
};