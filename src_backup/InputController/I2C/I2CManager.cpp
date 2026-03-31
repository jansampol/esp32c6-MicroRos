#include "InputController/I2C/I2CManager.h"

I2CManager::I2CManager(): 
    _pressureSensorMultiplexer(TCA_INTERNAL_ADDR),
    _externalMultiplexer1(TCA_EXTERNAL1_ADDR)//,
    //_externalMultiplexer2(TCA_EXTERNAL2_ADDR)
{

    // Do not initialize multiplexers here; do it after Wire.begin() in begin().
}


bool I2CManager::begin(uint8_t numberOfFerrisWheels, uint8_t otherSensors)
{
    Serial.printf("Starting I2C Manager with %d Ferris wheels and %d other sensors...\n", numberOfFerrisWheels, otherSensors);
    _numFerrisWheels = numberOfFerrisWheels;

    // Create I2C Bus
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    if(!Wire.begin()){
        Serial.println("I2C bus failed to start");
        return false;
    } else {
        Serial.println("I2C bus started");
    }

    // Now call begin() on each multiplexer
    if(!_pressureSensorMultiplexer.begin()){
        Serial.println("Pressure sensor multiplexer failed to initialize");
        return false;
    }
    if(!_externalMultiplexer1.begin()){
        Serial.println("External multiplexer 1 failed to initialize");
        return false;
    }
/*
    // No longer present, because two multiplexers are sufficient for two pressure sensors and 16 Ferris wheels
    if(!_externalMultiplexer2.begin()){
        Serial.println("External multiplexer 2 failed to initialize");
        return false;
    }
*/
    Serial.println("I2C multiplexers initialized. Initializing Ferris wheels...");
    bool ferrisResult = initFerrisWheels(numberOfFerrisWheels);

    if (ferrisResult){
        Serial.println("Ferris wheels successfully initialized.");
    } else {
        Serial.println("Failed to initialize ferris wheels");
        return false;
    }

    // set zero positions to the current values of the sensors
    for(uint8_t i = 0; i < _numFerrisWheels; i++){
        // this is without the mapping
        _rawPosition[i] = _ferrisWheelSensors[i].getCumulativePosition();
        _zeroOffset[i] = _rawPosition[i];
    }

    return true;
}

bool I2CManager::initFerrisWheels(uint8_t numberOfFerrisWheels){
    Serial.println("Initializing ferris wheel sensors...");
    // Allocate and initialize ferris wheel sensors
    if (numberOfFerrisWheels < 0 || numberOfFerrisWheels > MAX_FERRIS_WHEELS) {
        Serial.println("Number of ferris wheels cannot be negative!");
        return false;
    }

    _numFerrisWheels = numberOfFerrisWheels;

    // prepare angleDeg and ferris wheel sensor vector sizes
    _angleDeg.resize(_numFerrisWheels, 0.0f);
    _ferrisWheelSensors.resize(numberOfFerrisWheels, AS5600());

    for (uint8_t i = 0; i < numberOfFerrisWheels; i++) {

        // Create new AS5600 objects
        _ferrisWheelSensors[i] = AS5600();

        // Determine which multiplexer and channel to use
        if (i < 8) {
            // one of the first 8
            // select the correct channel using the wheelIdMapping
            _externalMultiplexer1.selectChannel(_wheelIdMapping[i]);

        } else if (i >= 8) {
            // one of the last 4 (depending on max ferris wheels)
            // select the correct channel using the wheelIdMapping
            //_externalMultiplexer2.selectChannel(_wheelIdMapping[i]);
            _pressureSensorMultiplexer.selectChannel(_wheelIdMapping[i]);

        } else {
            Serial.print("Multiplexer not available for ferris wheel ");
            Serial.println(i);
            return false;
        }

        // Initialize AS5600 sensors
        // check if there are more than one
        if(_ferrisWheelSensors.size() <= 0){
            Serial.println("No ferris wheel sensors to initialize");
            return false;
        } else {
            bool thisFerrisResult = _ferrisWheelSensors[i].begin();
            Serial.printf("Ferris wheel sensor %d: %d\n", i, thisFerrisResult);
            if (!thisFerrisResult) {
                Serial.print("initFerriswheels: Ferris wheel sensor ");
                Serial.print(i);
                Serial.println(" failed to initialize!");
                return false;
            }
        }
        delay(10);
    }
    // Serial.print("-    Init ");
    // Serial.print(_numFerrisWheels);
    // Serial.println(" ferris wheel sensors");
    return true;
}

float I2CManager::readFerrisWheelAngle(uint8_t wheelId){
    if(wheelId >= _numFerrisWheels || wheelId < 0){
        Serial.println("Invalid ferris wheel ID");
        return NAN;
    }

    // Select appropriate multiplexer channel
    if(wheelId < 8){
        // Serial.print("Selecting channel mux 1");
        // Serial.print(_wheelIdMapping[wheelId]);
        _externalMultiplexer1.selectChannel(_wheelIdMapping[wheelId]);
    
    } else {
        //_externalMultiplexer2.selectChannel(_wheelIdMapping[wheelId]);
        _pressureSensorMultiplexer.selectChannel(_wheelIdMapping[wheelId]);
    }


    _rawPosition[wheelId] = _ferrisWheelSensors[wheelId].getCumulativePosition();
    _angleDeg[wheelId] = (_rawPosition[wheelId] - _zeroOffset[wheelId]) * _scaleFactor[wheelId];

    // Serial.print("Ferris ");
    // Serial.print(wheelId);
    // Serial.print(" Angle: ");
    // Serial.print(_angleDeg[wheelId]);
    return _angleDeg[wheelId];
}

std::vector<float> I2CManager::readAllFerrisWheelAngles(){
    for(uint8_t i = 0; i < _numFerrisWheels; i++){
        // this stores it in the _angleDeg array
        _angleDeg[i] = readFerrisWheelAngle(i);
    }
    return _angleDeg;
}

float I2CManager::readFerrisWheelRawValue(uint8_t wheelId){
    if(wheelId >= _numFerrisWheels || wheelId < 0){
        Serial.println("Invalid ferris wheel ID");
        return NAN;
    }

    // Select appropriate multiplexer channel
    if(wheelId < 8){
        _externalMultiplexer1.selectChannel(_wheelIdMapping[wheelId]);
    } else {
        _pressureSensorMultiplexer.selectChannel(_wheelIdMapping[wheelId]);
    }

    // Return raw cumulative position without zero offset or scale conversion
    _rawPosition[wheelId] = _ferrisWheelSensors[wheelId].getCumulativePosition();
    return static_cast<float>(_rawPosition[wheelId] - _zeroOffset[wheelId]);
}

std::vector<float> I2CManager::readAllFerrisWheelRawValues(){
    std::vector<float> rawValues(_numFerrisWheels, 0.0f);
    for(uint8_t i = 0; i < _numFerrisWheels; i++){
        rawValues[i] = readFerrisWheelRawValue(i);
    }
    return rawValues;
}


/**
 * @brief Reads pressure in bar from the WF100DPZ10BG using TCA9548 multiplexer channel.
 * 
 * Steps:
 * - Uses TCA9548.selectChannel(channel) to route I2C to selected sensor.
 * - Polls sensor status register 0x02 for measurement readiness.
 * - Reads 3 bytes raw pressure data from register 0x06.
 * - Converts raw 24-bit signed integer to normalized float.
 * - Maps normalized value to calibrated pressure range (0-10 bar).
 * - Starts next measurement for continuous reading.
 * 
 * @param sensorId I2C multiplexer channel where sensor is connected.
 * @return float Pressure in bars or NAN if communication fails.
 */
float I2CManager::readPressureSensor(uint8_t sensorId){
    // check if sensorId is valid
    if(sensorId >= _numOfPressureSensors || sensorId < 0){
        Serial.println("Invalid pressure sensor ID");
        return NAN;
    } 

    // Select desired I2C channel on internal multiplexer of pressure sensor
    if (!_pressureSensorMultiplexer.selectChannel(sensorId)) {
        Serial.print("Failed to select I2C channel: ");
        Serial.println(sensorId);
        return NAN;
    }

    // Poll sensor status register (0x02) for measurement ready bit (bit0)
    Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(PRESSURE_SENSOR_ADDR, 1);

    bool statusOk = false;
    if (Wire.available()) {
        statusOk = Wire.read() & 0x01;
    }

    // If not ready, trigger a measurement and retry once
    if (!statusOk) {
        startPressureMeasurement();
        delay(10);

        Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
        Wire.write(0x02);
        Wire.endTransmission();
        Wire.requestFrom(PRESSURE_SENSOR_ADDR, 1);

        if (Wire.available()) {
            statusOk = Wire.read() & 0x01;
        } else {
            return NAN;
        }
    }

    if (!statusOk) {
        return NAN;
    }

    // Read 3-byte pressure register 0x06 MSB first
    Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
    Wire.write(0x06);
    Wire.endTransmission();
    Wire.requestFrom(PRESSURE_SENSOR_ADDR, 3);

    if (Wire.available() < 3) {
        return NAN;
    }

    // Collect 3 bytes of raw pressure data
    uint32_t rawPressure = 0;
    for (int i = 0; i < 3; i++) {
        rawPressure = (rawPressure << 8) | Wire.read();
    }

    // Start next measurement immediately after reading for continuous operation
    startPressureMeasurement();

    // Convert 24-bit signed integer raw pressure to normalized float
    float pressureNormalized = (rawPressure < 8388608)
        ? (float)rawPressure / 8388608.0f
        : ((float)rawPressure - 16777216.0f) / 8388608.0f;

    // Map normalized pressure value to calibrated pressure range (0-10 bar)
    float pressureBar = (pressureNormalized + 0.731f) * 10.0f;

    return pressureBar;
}

void I2CManager::startPressureMeasurement() {
    // Send command to start measurement on WF100DPZ10BG sensor
    Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
    Wire.write(0x30);      // Command register
    Wire.write(0x0A);      // Command to perform one output measurement
    Wire.endTransmission();
}
