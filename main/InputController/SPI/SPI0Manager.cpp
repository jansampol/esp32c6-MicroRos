#include "InputController/SPI/SPI0Manager.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "SPI0Manager";

// MCP23S17 base opcodes for HW address 000
static constexpr uint8_t MCP23S17_OPCODE_WRITE = 0x40;
static constexpr uint8_t MCP23S17_OPCODE_READ  = 0x41;

// MCP23S17 registers
static constexpr uint8_t MCP23S17_IODIRA = 0x00;
static constexpr uint8_t MCP23S17_IODIRB = 0x01;
static constexpr uint8_t MCP23S17_IPOLA  = 0x02;
static constexpr uint8_t MCP23S17_IPOLB  = 0x03;
static constexpr uint8_t MCP23S17_GPPUA  = 0x0C;
static constexpr uint8_t MCP23S17_GPPUB  = 0x0D;
static constexpr uint8_t MCP23S17_GPIOA  = 0x12;
static constexpr uint8_t MCP23S17_GPIOB  = 0x13;

SPI0Manager::SPI0Manager()
{
}

bool SPI0Manager::begin()
{
    if (_initialized) {
        return true;
    }

    ESP_LOGI(TAG, "SPI0Manager begin()");

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI0_MOSI_PIN;
    buscfg.miso_io_num = SPI0_MISO_PIN;
    buscfg.sclk_io_num = SPI0_CLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 16;

    ESP_LOGI(TAG, "Using pins MOSI=%d MISO=%d CLK=%d", SPI0_MOSI_PIN, SPI0_MISO_PIN, SPI0_CLK_PIN);

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000;  // safe starting point
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;                 // manual CS via demux
    devcfg.queue_size = 4;

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    err = initDemuxPins();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "initDemuxPins failed: %s", esp_err_to_name(err));
        return false;
    }

    deselectDevice();

    // ============================================================
    // MCP23S17 init - raw ESP-IDF version
    // ============================================================

    // Horizontal Buttons MCP23S17
    err = mcpInitMCP23S17(UI_1_HBUTTONS,
                          0xFFFF,   // all inputs
                          0xFFFF,   // pullups enabled
                          0xFFFF,   // polarity inverted
                          0x0000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Horizontal Buttons MCP23S17 init failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // LEDs and Switches MCP23S17
    err = mcpInitMCP23S17(UI_2_LEDS_AND_SWITCHES,
                          0xFFC0,   // upper bits input, lower bits output
                          0xFFC0,   // pullups on inputs
                          0xFFC0,   // invert inputs
                          0x0000);  // outputs low
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDs and Switches MCP23S17 init failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Vertical Buttons MCP23S17
    err = mcpInitMCP23S17(UI_3_VBUTTONS,
                          0xFFFF,
                          0xFFFF,
                          0xFFFF,
                          0x0000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Vertical Buttons MCP23S17 init failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Main Valve MCP23S17
    err = mcpInitMCP23S17(MAINVALVE,
                          0x0000,   // all outputs
                          0x0000,
                          0x0000,
                          0x0000);  // all low
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Main Valve MCP23S17 init failed: %s", esp_err_to_name(err));
        return false;
    }

    _mainValveState = 0x0000;
    _ledStates = 0x00;

    // ============================================================
    // Arduino-dependent parts intentionally disabled for now
    // ============================================================

    // ADC
    // selectDevice(ADC_POTMETERS);
    // _adcPotmeters.begin(FAKE_CS_PIN);
    // deselectDevice();

    // Screen
    // setScreenRES(false);
    // delay(1);
    // setScreenRES(true);
    // delay(1);
    // selectDevice(SCREEN);
    // _screen.init(240, 320);
    // _screen.setSPISpeed(80000000);
    // _screen.setRotation(1);
    // _screen.fillScreen(0x0000);
    // deselectDevice();
    // setScreenBLK(true);


    // LEDs and Switches MCP23S17
    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IODIRA, 0xFF); // inputs
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 IODIRA write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IODIRB, 0x00); // outputs
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 IODIRB write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPPUA, 0xFF); // pullups on A
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 GPPUA write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPPUB, 0x00); // no pullups on B
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 GPPUB write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IPOLA, 0xFF); // invert A inputs
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 IPOLA write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IPOLB, 0x00); // do not invert B
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 IPOLB write failed: %s", esp_err_to_name(err));
        return false;
    }

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOB, 0x00); // LEDs off initially
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UI_2 GPIOB init write failed: %s", esp_err_to_name(err));
        return false;
    }

    // Debug readback
    uint8_t iodirA = 0, iodirB = 0;
    uint8_t gpioA = 0, gpioB = 0;

    mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IODIRA, iodirA);
    mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_IODIRB, iodirB);
    mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, gpioA);
    mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOB, gpioB);

    ESP_LOGI(TAG, "UI_2 readback: IODIRA=0x%02X IODIRB=0x%02X GPIOA=0x%02X GPIOB=0x%02X",
            iodirA, iodirB, gpioA, gpioB);


    _initialized = true;
    ESP_LOGI(TAG, "SPI0Manager initialized.");
    return true;
}

esp_err_t SPI0Manager::initDemuxPins()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask =
        (1ULL << CS_1_PIN) |
        (1ULL << CS_2_PIN) |
        (1ULL << CS_3_PIN) |
        (1ULL << CS_4_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    deselectDevice();
    return ESP_OK;
}

void SPI0Manager::debugCSPins()
{
    for (int i = NO_DEVICE; i <= EXTERNAL_MCP_1; i++) {
        deviceNameSPI0 device = static_cast<deviceNameSPI0>(i);
        ESP_LOGI(TAG, "Selecting device: %d", i);
        selectDevice(device);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    deselectDevice();
}

void SPI0Manager::selectDevice(deviceNameSPI0 device)
{
    deselectDevice();
    esp_rom_delay_us(2);

    switch (device) {
        case NO_DEVICE:
            break;

        case UI_1_HBUTTONS:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case UI_2_LEDS_AND_SWITCHES:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case UI_3_VBUTTONS:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case MAINVALVE:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case ADC_POTMETERS:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case SCREEN:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 0);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        case EXTERNAL_MCP_1:
            gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 1);
            gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
            break;

        default:
            break;
    }

    _selectedDevice0 = device;
}

void SPI0Manager::deselectDevice()
{
    gpio_set_level(static_cast<gpio_num_t>(CS_1_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(CS_2_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(CS_3_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(CS_4_PIN), 0);
    _selectedDevice0 = NO_DEVICE;
}

esp_err_t SPI0Manager::mcpWrite8(deviceNameSPI0 device, uint8_t reg, uint8_t value)
{
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

    selectDevice(device);
    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    deselectDevice();

    return err;
}

esp_err_t SPI0Manager::mcpWrite16(deviceNameSPI0 device, uint8_t regA, uint16_t value)
{
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[4] = {
        MCP23S17_OPCODE_WRITE,
        regA,
        static_cast<uint8_t>(value & 0xFF),         // A
        static_cast<uint8_t>((value >> 8) & 0xFF)   // B
    };

    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;

    selectDevice(device);
    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    deselectDevice();

    return err;
}

esp_err_t SPI0Manager::mcpRead8(deviceNameSPI0 device, uint8_t reg, uint8_t& value)
{
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[3] = {
        MCP23S17_OPCODE_READ,
        reg,
        0x00
    };
    uint8_t rx[3] = {0, 0, 0};

    spi_transaction_t t = {};
    t.length = 24;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    selectDevice(device);
    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    deselectDevice();

    if (err == ESP_OK) {
        value = rx[2];
    }
    return err;
}

esp_err_t SPI0Manager::mcpRead16(deviceNameSPI0 device, uint8_t regA, uint16_t& value)
{
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[4] = {
        MCP23S17_OPCODE_READ,
        regA,
        0x00,
        0x00
    };
    uint8_t rx[4] = {0, 0, 0, 0};

    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    selectDevice(device);
    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    deselectDevice();

    if (err == ESP_OK) {
        value = static_cast<uint16_t>(rx[2]) |
                (static_cast<uint16_t>(rx[3]) << 8);
    }
    return err;
}

esp_err_t SPI0Manager::adcReadMcp3004(uint8_t channel, uint16_t& value)
{
    if (_spi_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCP3004 single-ended read:
    // tx[0] = start bit, tx[1] = SGL/DIFF + channel selection, tx[2] = dummy
    uint8_t tx[3] = {
        0x01,
        static_cast<uint8_t>((0x08U | channel) << 4),
        0x00
    };
    uint8_t rx[3] = {0, 0, 0};

    spi_transaction_t t = {};
    t.length = 24;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    selectDevice(ADC_POTMETERS);
    esp_err_t err = spi_device_transmit(_spi_dev, &t);
    deselectDevice();

    if (err == ESP_OK) {
        value = static_cast<uint16_t>(((rx[1] & 0x03U) << 8) | rx[2]);
    }
    return err;
}

esp_err_t SPI0Manager::mcpInitMCP23S17(deviceNameSPI0 device,
                                       uint16_t iodir,
                                       uint16_t gppu,
                                       uint16_t ipol,
                                       uint16_t initial_gpio)
{
    esp_err_t err = mcpWrite16(device, MCP23S17_IODIRA, iodir);
    if (err != ESP_OK) return err;

    err = mcpWrite16(device, MCP23S17_GPPUA, gppu);
    if (err != ESP_OK) return err;

    err = mcpWrite16(device, MCP23S17_IPOLA, ipol);
    if (err != ESP_OK) return err;

    err = mcpWrite16(device, MCP23S17_GPIOA, initial_gpio);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

uint16_t SPI0Manager::readHorizontalButtons()
{
    uint16_t value = 0;
    esp_err_t err = mcpRead16(UI_1_HBUTTONS, MCP23S17_GPIOA, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readHorizontalButtons failed: %s", esp_err_to_name(err));
        return 0;
    }
    return swapLastHorizontalButtonPairs(value);
}

uint16_t SPI0Manager::swapLastHorizontalButtonPairs(uint16_t bits)
{
    uint16_t lowerHalf = bits & 0b0000000011111111;
    uint16_t evenBits  = (bits & 0b0101010100000000) << 1;
    uint16_t oddBits   = (bits & 0b1010101000000000) >> 1;
    return lowerHalf | evenBits | oddBits;
}

uint16_t SPI0Manager::readVerticalButtons()
{
    uint16_t value = 0;
    esp_err_t err = mcpRead16(UI_3_VBUTTONS, MCP23S17_GPIOA, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readVerticalButtons failed: %s", esp_err_to_name(err));
        return 0;
    }
    return value;
}

uint8_t SPI0Manager::invertByte(uint8_t b)
{
    uint8_t outp = 0;
    for (int i = 0; i < 8; i++) {
        if ((b >> i) & 0x01) {
            outp |= (1U << (7 - i));
        }
    }
    return outp;
}

uint8_t SPI0Manager::readSwitches()
{
    uint8_t value = 0;
    esp_err_t err = mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readSwitches failed: %s", esp_err_to_name(err));
        return 0;
    }
    return invertByte(value);
}

void SPI0Manager::writeGreen(bool on)  { setLedBit(0, on); }
void SPI0Manager::writeRed1(bool on)   { setLedBit(1, on); }
void SPI0Manager::writeRed2(bool on)   { setLedBit(2, on); }
void SPI0Manager::writeRed3(bool on)   { setLedBit(3, on); }
void SPI0Manager::writeYellow(bool on) { setLedBit(4, on); }
void SPI0Manager::writeBlue(bool on)   { setLedBit(5, on); }

void SPI0Manager::setLedBit(uint8_t ledIndex, bool on)
{
    if (ledIndex > 5) return;

    uint8_t mask = (1U << ledIndex);
    if (on) _ledStates |= mask;
    else    _ledStates &= ~mask;

    writeLeds(_ledStates);
}

void SPI0Manager::writeLeds(uint8_t ledStates)
{
    _ledStates = ledStates & 0x3F;  // 6 LEDs

    esp_err_t err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOB, _ledStates);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "writeLeds failed: %s", esp_err_to_name(err));
    }
}

void SPI0Manager::setScreenRES(bool on)
{
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_RES)
                         : (_mainValveState & ~MAIN_VALVE_RES);
    writeMainValve(_mainValveState);
}

void SPI0Manager::setScreenBLK(bool on)
{
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_BLK)
                         : (_mainValveState & ~MAIN_VALVE_BLK);
    writeMainValve(_mainValveState);
}

// void SPI0Manager::setMainValve(bool on)
// {
//     _mainValveState = on ? (_mainValveState | MAIN_VALVE_VALVE)
//                          : (_mainValveState & ~MAIN_VALVE_VALVE);
//     writeMainValve(_mainValveState);
// }

void SPI0Manager::setMainValve(bool on)
{
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_VALVE)
                         : (_mainValveState & ~MAIN_VALVE_VALVE);
    writeMainValve(_mainValveState);
}

void SPI0Manager::writeMainValve(uint16_t state)
{
    _mainValveState = state;

    esp_err_t err = mcpWrite16(MAINVALVE, MCP23S17_GPIOA, state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "writeMainValve failed: %s", esp_err_to_name(err));
    }
}

uint16_t SPI0Manager::readPotmeters(bool id)
{
    const uint8_t channel = id ? 1U : 0U;
    uint16_t raw = 0;
    esp_err_t err = adcReadMcp3004(channel, raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readPotmeters(ch=%u) failed: %s", channel, esp_err_to_name(err));
        return 0;
    }

    // Keep compatibility with previous behavior from Arduino code.
    return static_cast<uint16_t>(1023U - raw);
}

uint16_t SPI0Manager::readExternalDevice(deviceNameSPI0 device)
{
    if (device == EXTERNAL_MCP_1) {
        uint16_t value = 0;
        esp_err_t err = mcpRead16(EXTERNAL_MCP_1, MCP23S17_GPIOA, value);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "readExternalDevice failed: %s", esp_err_to_name(err));
            return 0;
        }
        return value;
    }
    return 0;
}

void SPI0Manager::writeExternalDevice(deviceNameSPI0 device, uint16_t data)
{
    if (device == EXTERNAL_MCP_1) {
        esp_err_t err = mcpWrite16(EXTERNAL_MCP_1, MCP23S17_GPIOA, data);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "writeExternalDevice failed: %s", esp_err_to_name(err));
        }
    }
}

void SPI0Manager::debugReadMcpRegisters(deviceNameSPI0 device)
{
    uint8_t v = 0;

    if (mcpRead8(device, MCP23S17_IODIRA, v) == ESP_OK) {
        ESP_LOGI(TAG, "Device %d IODIRA = 0x%02X", device, v);
    } else {
        ESP_LOGE(TAG, "Device %d read IODIRA failed", device);
    }

    if (mcpRead8(device, MCP23S17_IODIRB, v) == ESP_OK) {
        ESP_LOGI(TAG, "Device %d IODIRB = 0x%02X", device, v);
    } else {
        ESP_LOGE(TAG, "Device %d read IODIRB failed", device);
    }

    if (mcpRead8(device, MCP23S17_GPIOA, v) == ESP_OK) {
        ESP_LOGI(TAG, "Device %d GPIOA = 0x%02X", device, v);
    } else {
        ESP_LOGE(TAG, "Device %d read GPIOA failed", device);
    }

    if (mcpRead8(device, MCP23S17_GPIOB, v) == ESP_OK) {
        ESP_LOGI(TAG, "Device %d GPIOB = 0x%02X", device, v);
    } else {
        ESP_LOGE(TAG, "Device %d read GPIOB failed", device);
    }
}

void SPI0Manager::debugWriteAndReadLeds(uint8_t value)
{
    value &= 0x3F; // only LED bits

    uint8_t current = 0;
    esp_err_t err = mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, current);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read before write failed");
        return;
    }

    // Preserve switch bits (A6, A7)
    uint8_t newValue = (current & 0xC0) | value;

    err = mcpWrite8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, newValue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write failed");
        return;
    }

    uint8_t readback = 0;
    mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, readback);

    ESP_LOGI(TAG, "GPIOA write=0x%02X read=0x%02X", newValue, readback);
}

void SPI0Manager::debugReadRawDevice2(uint8_t& gpioA, uint8_t& gpioB)
{
    gpioA = 0;
    gpioB = 0;

    if (mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOA, gpioA) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device 2 GPIOA");
    }

    if (mcpRead8(UI_2_LEDS_AND_SWITCHES, MCP23S17_GPIOB, gpioB) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device 2 GPIOB");
    }
}
