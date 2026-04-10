#pragma once

// =====================================================
// ESP32-C6 pin draft
// IMPORTANT: still verify against your exact board wiring
// =====================================================

// SPI for motor valves
#define SPI1_MISO_PIN 20
#define SPI1_MOSI_PIN 19
#define SPI1_CLK_PIN  15
#define SPI1_CS_MOTOR 23

// SPI0 (other spi devices, internal and external)
#ifdef JULIAN_GRACE_CONFIG
#define SPI0_MISO_PIN 0   // C5 GPIO6 position -> C6 GPIO0
#else
#define SPI0_MISO_PIN 6   // C5 GPIO0 position -> C6 GPIO6
#endif

#define SPI0_CLK_PIN 1    // C5 GPIO7 position -> C6 GPIO1
#define SPI0_MOSI_PIN 8   // C5 GPIO8 position -> C6 GPIO8

// CS demuxer pins
#define CS_1_PIN 3
#define CS_2_PIN 2
#define CS_3_PIN 21
#define CS_4_PIN 9

// I2C
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// Changed from invalid ESP32-C5/ESP32 mappings
#define EMERGENCY_PIN 2
#define SCREEN_DC_PIN 3

#define FAKE_CS_PIN 13

// Main valve GPIO expander
#define MAIN_VALVE_VALVE 0x01
#define MAIN_VALVE_RES   0x02
#define MAIN_VALVE_BLK   0x04

// // SPI1 (MOTOR VALVES)
// #define SPI1_MISO_PIN 19
// #define SPI1_MOSI_PIN 23
// #define SPI1_CLK_PIN 18
// #define SPI1_CS_MOTOR 5

// // SPI0 (other spi devices, internal and external)
// #ifdef JULIAN_GRACE_CONFIG
// #define SPI0_MISO_PIN 6
// #else
// #define SPI0_MISO_PIN 16
// #endif

// #define SPI0_CLK_PIN 4
// #define SPI0_MOSI_PIN 17

// // Bit pins for CS demuxer (these should be floating on boot)
// #define CS_1_PIN 25
// #define CS_2_PIN 26
// #define CS_3_PIN 27
// #define CS_4_PIN 14

// // I2C pins
// #define I2C_SDA_PIN 21
// #define I2C_SCL_PIN 22

// // #define ADC_PIN 9
// #define EMERGENCY_PIN 32
// #define SCREEN_DC_PIN 33

// #define FAKE_CS_PIN 13 // or 6

// // main valve GPIO expander
// #ifdef JULIAN_GRACE_CONFIG
//     #define MAIN_VALVE_VALVE 0x04 // GPB3 = pin 10 // MCP_MAIN.write1(10, r);
//     #define MAIN_VALVE_RES 0x100  // GPA0 = pin 0  // MCP_MAIN.write1(0, r);
//     #define MAIN_VALVE_BLK 0x200  // GPA1 = pin 1  // MCP_MAIN.write1(1, r);
// #else
//     #define MAIN_VALVE_VALVE 0x01 // GPB0 _mainValve.write8(1, state); // port B
//     #define MAIN_VALVE_RES 0x02   // GPB1
//     #define MAIN_VALVE_BLK 0x04   // GPB2
// #endif