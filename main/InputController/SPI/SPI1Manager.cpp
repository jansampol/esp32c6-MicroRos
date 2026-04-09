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
    ESP_LOGI(TAG, "SPI1Manager begin()");
    ESP_LOGI(TAG, "Using pins MOSI=%d MISO=%d CLK=%d CS=%d",
             SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_CLK_PIN, SPI1_CS_MOTOR);

    if (_initialized) {
        ESP_LOGI(TAG, "Already initialized");
        return true;
    }

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI1_MOSI_PIN;
    buscfg.miso_io_num = SPI1_MISO_PIN;
    buscfg.sclk_io_num = SPI1_CLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 16;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_bus_initialize -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        return false;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = SPI1_CS_MOTOR;
    devcfg.queue_size = 4;

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi_dev);
    ESP_LOGI(TAG, "spi_bus_add_device -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        return false;
    }

    err = mcpInit();
    ESP_LOGI(TAG, "mcpInit -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        return false;
    }

    _initialized = true;
    ESP_LOGI(TAG, "SPI1Manager initialized OK");
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
        ESP_LOGE(TAG, "mcpWrite16: _spi_dev is null");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[4] = {
        MCP23S17_OPCODE_WRITE,
        regA,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };

    ESP_LOGI(TAG,
             "mcpWrite16 reg=0x%02X value=0x%04X tx=[0x%02X 0x%02X 0x%02X 0x%02X]",
             regA, value, tx[0], tx[1], tx[2], tx[3]);

    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;

    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    ESP_LOGI(TAG, "spi_device_transmit -> %s", esp_err_to_name(err));
    return err;
}

esp_err_t SPI1Manager::mcpInit() {
    ESP_LOGI(TAG, "mcpInit: configuring IODIR");
    esp_err_t err = mcpWrite16(MCP23S17_IODIRA, 0x0000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpInit: IODIR write failed");
        return err;
    }

    ESP_LOGI(TAG, "mcpInit: clearing GPIO");
    err = mcpWrite16(MCP23S17_GPIOA, 0x0000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpInit: GPIO clear failed");
        return err;
    }

    _valveStates = 0;
    return ESP_OK;
}

void SPI1Manager::writeValves(uint16_t valveStates) {
    _valveStates = valveStates;

    if (!_initialized || _spi_dev == nullptr) {
        ESP_LOGE(TAG, "writeValves ignored: initialized=%d spi_dev=%p",
                 _initialized, _spi_dev);
        return;
    }

    ESP_LOGI(TAG, "writeValves 0x%04X", valveStates);

    esp_err_t err = mcpWrite16(MCP23S17_GPIOA, valveStates);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "writeValves failed: %s", esp_err_to_name(err));
    }
}

uint16_t SPI1Manager::getValveStates() {
    return _valveStates;
}