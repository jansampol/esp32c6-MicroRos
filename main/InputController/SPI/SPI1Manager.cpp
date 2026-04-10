#include "SPI1Manager.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "SPI1Manager";

// MCP23S17 base opcodes for HW address 000
static constexpr uint8_t MCP23S17_OPCODE_WRITE = 0x40;
static constexpr uint8_t MCP23S17_OPCODE_READ  = 0x41;

// registers
static constexpr uint8_t MCP23S17_IODIRA = 0x00;
static constexpr uint8_t MCP23S17_IODIRB = 0x01;
static constexpr uint8_t MCP23S17_GPIOA  = 0x12;
static constexpr uint8_t MCP23S17_GPIOB  = 0x13;

SPI1Manager::SPI1Manager() {
}

bool SPI1Manager::begin() {
    if (_initialized) {
        return true;
    }

    ESP_LOGI(TAG, "SPI1Manager begin()");

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI1_MOSI_PIN;
    buscfg.miso_io_num = SPI1_MISO_PIN;
    buscfg.sclk_io_num = SPI1_CLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 16;

    ESP_LOGI(TAG, "Using pins MOSI=%d MISO=%d CLK=%d CS=%d",
             SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_CLK_PIN, SPI1_CS_MOTOR);

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000;   // safe starting point
    devcfg.mode = 0;
    devcfg.spics_io_num = SPI1_CS_MOTOR;
    devcfg.queue_size = 4;

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    if (mcpInit() != ESP_OK) {
        ESP_LOGE(TAG, "MCP23S17 init failed");
        return false;
    }

    _initialized = true;
    return true;
}

esp_err_t SPI1Manager::mcpWrite8(uint8_t reg, uint8_t value) {
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[3] = {
        MCP23S17_OPCODE_WRITE,
        reg,
        value
    };

    spi_transaction_t t = {};
    t.length = 24;
    t.tx_buffer = tx;

    return spi_device_transmit(_spi_dev, &t);
}

esp_err_t SPI1Manager::mcpWrite16(uint8_t regA, uint16_t value) {
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[4] = {
        MCP23S17_OPCODE_WRITE,
        regA,
        static_cast<uint8_t>(value & 0xFF),         // GPIOA
        static_cast<uint8_t>((value >> 8) & 0xFF)   // GPIOB
    };

    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;

    return spi_device_transmit(_spi_dev, &t);
}

esp_err_t SPI1Manager::mcpInit() {
    // all 16 pins as outputs
    esp_err_t err = mcpWrite16(MCP23S17_IODIRA, 0x0000);
    if (err != ESP_OK) {
        return err;
    }

    // set all outputs low
    err = mcpWrite16(MCP23S17_GPIOA, 0x0000);
    if (err != ESP_OK) {
        return err;
    }

    _valveStates = 0;
    return ESP_OK;
}

void SPI1Manager::writeValves(uint16_t valveStates) {
    _valveStates = valveStates;
    if (!_initialized || _spi_dev == nullptr) {
        return;
    }

    esp_err_t err = mcpWrite16(MCP23S17_GPIOA, valveStates);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "writeValves failed: %s", esp_err_to_name(err));
    }
}

uint16_t SPI1Manager::getValveStates() {
    return _valveStates;
}