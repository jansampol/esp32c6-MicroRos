/*
    This class manages the SPI0 bus, used to communicate with internal & external spi devices for hardware. (Except motor drivers) 
    This bus uses a demultiplexer to select between multiple devices
    With 4 i/o pins 16 chip select (CS) lines can be controlled. 
    
    Make sure to always first select the device, then update it using its library functions, then deselect the device again.

    Before any SPI transfer we have to quickly turn on and off the CS pin. (At least for MCP23S17)

    An enum is used to identify each device with a name instead of a number.

    The class uses edited libraries for the MCP23S17 and MCP3XXX ADC 
    to allow for the manual control of the CS pin as this is required for expansion.

    The screen library (Adafruit_SPITFT) also needs to be modified to allow for no CS pin in the constructor.
*/

#pragma once
#include <Arduino.h>
#include "PinDefinitions.h"     
#include <SPI.h>

#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_GFX.h>    // Core graphics library
// #include <Fonts/FreeSans12pt7b.h> // 18 high; one line is 29 pixels
// #include "hsv2rgb.h"

// #define HEADER_COLOR (tft.color565(0, 255, 0))


//#include "InputController/SPI/MCP23S17_v3.h"
//#include "InputController/SPI/MCP3XXX.h"
#include <MCP23S17.h>
#include <MCP3XXX.h>
#include "SystemParameters.h"



#define NUM_SPI0_DEVICES 7
enum deviceNameSPI0 {
    NO_DEVICE = 0,
    UI_1_HBUTTONS = 1,
    UI_2_LEDS_AND_SWITCHES = 2,
    UI_3_VBUTTONS = 3,
    MAINVALVE = 4,
    ADC_POTMETERS = 5,
    SCREEN = 6,
    EXTERNAL_MCP_1 = 7,
};

class SPI0Manager{
    public:

        // Constructor
        SPI0Manager();

        bool begin();
        void debugCSPins();

        // void setPinmode();


        void selectDevice(deviceNameSPI0 device);
        void deselectDevice();

        void setMainValve(bool on);
        void setScreenRES(bool on);
        void setScreenBLK(bool on);
        void writeMainValve(uint16_t state);

        uint16_t readHorizontalButtons();
        uint16_t readVerticalButtons();
        uint8_t readSwitches();

        void writeGreen(bool on);
        void writeRed1(bool on);
        void writeRed2(bool on);
        void writeRed3(bool on);
        void writeYellow(bool on);
        void writeBlue(bool on);
        void writeLeds(uint8_t ledStates);
        
        uint16_t readPotmeters(bool id);
        
        // Accept non-owning pointer to canvas owned by InputController
        // void updateScreen(GFXcanvas1 * canvas);

        // pass reference to screen to the canvas manager / screen manager
        Adafruit_ST7789& getScreen() { return _screen; }

        // External SPI Devices can be added here
        uint16_t readExternalDevice(deviceNameSPI0 device);
        void writeExternalDevice(deviceNameSPI0 device, uint16_t data);

        
    private:
        uint8_t invertByte(uint8_t b);

        // Swaps the last 4 columns of horizontal buttons. The buttons are
        // physically swapped on the PCB. This fixes the logical implementation
        uint16_t swapLastHorizontalButtonPairs(uint16_t bits);
        
        uint8_t _ledStates = 0;
        void setLedBit(uint8_t bitIndex, bool on);

        uint16_t _mainValveState = 0;

        deviceNameSPI0 _selectedDevice0 = NO_DEVICE;

        MCP23S17 _mainValve;
        MCP23S17 _horizontalButtons;
        MCP23S17 _ledsAndSwitches;
        MCP23S17 _verticalButtons;
        MCP23S17 _externalMCP1;

        MCP3004 _adcPotmeters;
        Adafruit_ST7789 _screen;

        // add external devices here (make sure the libraries are modified so they 
        // allow for no CS. we have to handle this ourselves in this wrapper class)
        // These classes dont know about the demultiplexer weve added.
};