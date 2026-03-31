/*  
    This class manages I2C communication with all TCA9548 multiplexers, 
    The internal TCA9548 multiplexer has two pressure sensors connected. (WF100DPZ10BG pressure sensors)
    Up to two external TCA9548 multiplexers can be connected to read up to 16 Ferriswheel sensors. (AS5600 magnetic rotary encoder)

    Use the begin() function to initialize the I2C bus with correct number of multiplexers and ferris wheels.
    Therefore dynamic allocation is used.
*/

#pragma once
#include <Arduino.h>
#include "PinDefinitions.h"    
// #include <Wire.h>
#include "TCA9548.h" 
#include "AS5600.h"
#include <vector>

#define TCA_INTERNAL_ADDR 0x70      // internal multiplexer     (A0 low, A1 low, A2 low)
#define TCA_EXTERNAL1_ADDR 0x71      // external multiplexer    (A0 high, A1 low, A2 low)
//#define TCA_EXTERNAL2_ADDR 0x72      // external multiplexer    (A0 low, A1 high, A2 low)

#define PRESSURE_SENSOR_ADDR 0x6D

// when more ferris wheels are needed, change this value and add channels in the _wheelIdMapping array
// for now last 4 are reserved for other I2C sensors, these do need functions to read them
#define MAX_FERRIS_WHEELS 14

// does nothing yet
#define EXT_I2C_DEVICES 4


class I2CManager{
    public:
        // Constructor
        I2CManager();

        // Create I2C bus, init TCA multiplexers, and ferris wheel sensors
        bool begin(uint8_t numberOfFerrisWheels, uint8_t otherSensors = 0);
        bool initFerrisWheels(uint8_t numberOfFerrisWheels);

        float readFerrisWheelAngle(uint8_t wheelId);
        std::vector<float> readAllFerrisWheelAngles();
        
        float readFerrisWheelRawValue(uint8_t wheelId);
        std::vector<float> readAllFerrisWheelRawValues();

        float readPressureSensor(uint8_t sensorId);
        void startPressureMeasurement();

        uint8_t getNumOfFerrisWheels(){ return _numFerrisWheels; }
        uint8_t getNumOfPressureSensors(){ return _numOfPressureSensors; }


    private:
        uint8_t _numFerrisWheels = 0;
        const uint8_t _numOfPressureSensors = 2;

        // This mapping converts Wheelid to the correct channel
        // The last 4 are reserved for other I2C sensors
        const uint8_t _wheelIdMapping[14] = {
            0, 1, 2, 3, 4, 5, 6, 7,     // external mux 1
            2, 3, 4, 5, 6, 7            // pressure sensor mux
        };

        // I2C Multiplexers (constant amount)
        TCA9548 _pressureSensorMultiplexer;
        TCA9548 _externalMultiplexer1;
        //TCA9548 _externalMultiplexer2;

        // Ferris Wheel Sensors
        std::vector<AS5600> _ferrisWheelSensors;    

        // Calibrate Ferris wheels
        int32_t _rawPosition[16];
        std::vector<float> _angleDeg;

        // set to the postion at startup
        int32_t _zeroOffset[16] = {
            0, 0, 0, 0,     
            0, 0, 0, 0,     
            0, 0, 0, 0,     
            0, 0, 0, 0
        };
        
        // used for calibration of ferris wheels (Wheel ID stays the same, order can be changed depending on connection)
        const float _scaleFactor[16] = {
            (360.0/4096.0),        // 0
            (360.0/4096.0),        // 1
            (360.0/4096.0),        // 2
            (360.0/4096.0),        // 3
            (360.0/4096.0),        // 4
            (360.0/4096.0),        // 5    
            (360.0/4096.0),        // 6
            (360.0/4096.0),        // 7
            (360.0/4096.0),        // 8
            (360.0/4096.0),        // 9
            (360.0/4096.0),        // 10
            (360.0/4096.0),        // 11
            (360.0/4096.0),        // 12
            (360.0/4096.0),        // 13
            (360.0/4096.0),        // 14
            (360.0/4096.0),        // 15
        };

};