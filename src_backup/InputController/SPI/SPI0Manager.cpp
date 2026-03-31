#include "InputController/SPI/SPI0Manager.h"

SPI0Manager::SPI0Manager() :
    // Initialize MCP23S17 instances (must follow declaration order)
    /*
    _mainValve(SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, 0x00),
    _horizontalButtons(SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, 0x00),
    _ledsAndSwitches(SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, 0x00),
    _verticalButtons(SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, 0x00),
    _externalMCP1(SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, 0x00),
    _adcPotmeters(),
    */
    //_screen(CS_4_PIN, SCREEN_DC_PIN, SPI0_MOSI_PIN, SPI0_CLK_PIN, -1) // Software SPI, too slow

    _mainValve(FAKE_CS_PIN),
    _horizontalButtons(FAKE_CS_PIN),
    _ledsAndSwitches(FAKE_CS_PIN),
    _verticalButtons(FAKE_CS_PIN),
    _externalMCP1(FAKE_CS_PIN),
    _adcPotmeters(),
    _screen(&SPI, CS_4_PIN, SCREEN_DC_PIN, -1) // Screen requires hardware SPI for best performance
{

}

bool SPI0Manager::begin(){
    Serial.println("Starting SPI...");
    SPI.begin(
        SPI0_CLK_PIN,   // SCK
        SPI0_MISO_PIN,  // MISO (not used by ST7789, but required)
        SPI0_MOSI_PIN,  // MOSI
        -1
    );    

    // debugCSPins();
    // Before any SPI transfer we have to quickly turn on and off the CS pin. (At least for MCP23S17 with modified lib)
    // The function select device does this toggle action.

    deselectDevice();
    // Select Horizontal Buttons MCP and init
    selectDevice(UI_1_HBUTTONS);
    if(!_horizontalButtons.begin()){
        Serial.println("Horizontal Buttons MCP23S17 failed to initialize!");
        return false;
    }
    selectDevice(UI_1_HBUTTONS);
    _horizontalButtons.pinMode16(0xffff);
    selectDevice(UI_1_HBUTTONS);
    _horizontalButtons.setPullup16(0xffff);
    selectDevice(UI_1_HBUTTONS);
    _horizontalButtons.setPolarity16(0xffff);
    deselectDevice();


    // Serial.print("Horizontal Buttons: ");
    // Serial.println(readHorizontalButtons(), BIN);

    delay(10);

    // setup MCP for leds and switches
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    if(!_ledsAndSwitches.begin()){
        Serial.println("LEDs and Switches MCP23S17 failed to initialize!");
        return false;
    }
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    _ledsAndSwitches.pinMode16(0xffc0);
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    _ledsAndSwitches.setPullup16(0xffc0);
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    _ledsAndSwitches.setPolarity16(0xffc0);
    deselectDevice();

    delay(10);

    // setup MCP for vertical buttons
    selectDevice(UI_3_VBUTTONS);
    if(!_verticalButtons.begin()){
        Serial.println("Vertical Buttons MCP23S17 failed to initialize!");
        return false;
    }
    selectDevice(UI_3_VBUTTONS);
    _verticalButtons.pinMode16(0xffff);
    selectDevice(UI_3_VBUTTONS);
    _verticalButtons.setPullup16(0xffff);
    selectDevice(UI_3_VBUTTONS);
    _verticalButtons.setPolarity16(0xffff);
    deselectDevice();
    
    delay(10);

    // setup MCP for main valve
    selectDevice(MAINVALVE);
    if(!_mainValve.begin()){
        Serial.println("Main Valve MCP23S17 failed to initialize!");
        return false;
    }
    selectDevice(MAINVALVE);
    _mainValve.pinMode16(0x0000); // all outputs
    selectDevice(MAINVALVE);
    _mainValve.write16(0x0000); // all low
    deselectDevice();


    // setup ADC
    selectDevice(ADC_POTMETERS);
    _adcPotmeters.begin(FAKE_CS_PIN);
    deselectDevice();

    // Setup screen
    Serial.println("Initializing screen in SPI0Manager");

    // Hardware reset
    setScreenRES(false); // uses MAINVALVE = 4
    delay(1);
    setScreenRES(true); // release reset
    delay(1);
    
    // Select screen and keep it selected during all operations
    selectDevice(SCREEN);
    delay(1);
    Serial.println("Screen device selected, calling init...");
    _screen.init(240, 320);
    _screen.setSPISpeed(80000000);
    _screen.setRotation(1);
    _screen.fillScreen(0x0000); // black
    deselectDevice();
    setScreenBLK(true); // set backlight on only after filling it 
    
    Serial.println("Screen initialized by SPI0Manager.");

    // setup External MCP 1 (Change pinmodes and settings as needed.)
    // selectDevice(EXTERNAL_MCP_1);
    // if(!_externalMCP1.begin()){
    //     Serial.println("External MCP 1 failed to initialize!");
    //     return false;
    // }
    // selectDevice(EXTERNAL_MCP_1);
    // _externalMCP1.pinMode16(0xffff);
    // selectDevice(EXTERNAL_MCP_1);
    // _externalMCP1.write16(0x0000);
    // deselectDevice();
    
    delay(10);

    return true;
}

void SPI0Manager::debugCSPins(){
    // select each device, print a message, and wait for 5 seconds.
    for(int i = NO_DEVICE; i <= EXTERNAL_MCP_1; i++){
        deviceNameSPI0 device = static_cast<deviceNameSPI0>(i);
        Serial.print("Selecting device: ");
        Serial.println(i);
        SPI0Manager spiManager;
        spiManager.selectDevice(device);
        delay(5000);
    }
}

void SPI0Manager::selectDevice(deviceNameSPI0 device){
    // CS pin has to be toggles, so select no device first and then retrigger cs.
    deselectDevice();
    delayMicroseconds(2);

    switch(device){
        case NO_DEVICE:
            // select 0 in binary 0000
            // do nothing as all pins are already low
            break;
        case UI_1_HBUTTONS:
            // select 1 in binary 0001
            digitalWrite(CS_1_PIN, HIGH);
            digitalWrite(CS_2_PIN, LOW);
            digitalWrite(CS_3_PIN, LOW);
            digitalWrite(CS_4_PIN, LOW);
            break;
        case UI_2_LEDS_AND_SWITCHES:
            // select 2 in binary 0010
            digitalWrite(CS_1_PIN, LOW);    
            digitalWrite(CS_2_PIN, HIGH);
            digitalWrite(CS_3_PIN, LOW);
            digitalWrite(CS_4_PIN, LOW);
            break;
        case UI_3_VBUTTONS:
            // select 3 in binary 0011
            digitalWrite(CS_1_PIN, HIGH);
            digitalWrite(CS_2_PIN, HIGH);
            digitalWrite(CS_3_PIN, LOW);
            digitalWrite(CS_4_PIN, LOW);
            break;
        case MAINVALVE:
            // select 4 in binary 0100
            digitalWrite(CS_1_PIN, LOW);
            digitalWrite(CS_2_PIN, LOW);
            digitalWrite(CS_3_PIN, HIGH);
            digitalWrite(CS_4_PIN, LOW);
            break;
        case ADC_POTMETERS:
            // select 5 in binary 0101
            digitalWrite(CS_1_PIN, HIGH);
            digitalWrite(CS_2_PIN, LOW);
            digitalWrite(CS_3_PIN, HIGH);
            digitalWrite(CS_4_PIN, LOW);
            break;
        case SCREEN:
            // select 6 in binary 0110
            digitalWrite(CS_1_PIN, LOW);
            digitalWrite(CS_2_PIN, HIGH);
            digitalWrite(CS_3_PIN, HIGH);
            digitalWrite(CS_4_PIN, LOW);
            break;
        // To add external devices, add to enum in .h file and make another case here with correct binary mapping.
        case EXTERNAL_MCP_1:
            // select 7 in binary 0111
            digitalWrite(CS_1_PIN, HIGH);
            digitalWrite(CS_2_PIN, HIGH);
            digitalWrite(CS_3_PIN, HIGH);
            digitalWrite(CS_4_PIN, LOW);
            break;
        default:
            // deselect all
            digitalWrite(CS_1_PIN, LOW);
            digitalWrite(CS_2_PIN, LOW);
            digitalWrite(CS_3_PIN, LOW);
            digitalWrite(CS_4_PIN, LOW);
            break;
    }
}

void SPI0Manager::deselectDevice(){
    // 'select' NO_DEVICE > where all 4 bits are low
    digitalWrite(CS_1_PIN, LOW);
    digitalWrite(CS_2_PIN, LOW);
    digitalWrite(CS_3_PIN, LOW);
    digitalWrite(CS_4_PIN, LOW);
}

uint16_t SPI0Manager::readHorizontalButtons(){
    selectDevice(UI_1_HBUTTONS);
    uint16_t value =  swapLastHorizontalButtonPairs(_horizontalButtons.read16());
    deselectDevice();
    return value;
}

uint16_t SPI0Manager::swapLastHorizontalButtonPairs(uint16_t bits) {
    // Keep the lower 8 bits
    uint16_t lowerHalf = bits & 0b0000000011111111;

    // Bit mask the 8 higher odd bits and shift them left
    uint16_t evenBits = (bits & 0b0101010100000000) << 1;

    // Bit mask the 8 lower even bits and shift them right
    uint16_t oddBits = (bits & 0b1010101000000000) >> 1;

    return lowerHalf | evenBits | oddBits;
}

uint16_t SPI0Manager::readVerticalButtons(){
    selectDevice(UI_3_VBUTTONS);
    uint16_t value =  _verticalButtons.read16();
    deselectDevice();
    return value;
}

uint8_t SPI0Manager::invertByte(uint8_t b) {
  uint8_t outp = 0;  // top-down
  for (int i = 0; i < 8; i++) {
    if (bitRead(b, i)) {
      outp = bitSet(outp, 7 - i);
    }
  }
  return outp;
}

uint8_t SPI0Manager::readSwitches() {
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    uint8_t value = invertByte(_ledsAndSwitches.read8(0));
    deselectDevice();
    return value;
}

void SPI0Manager::writeGreen(bool on){
    setLedBit(0, on);
}
void SPI0Manager::writeRed1(bool on){
    setLedBit(1, on);
}
void SPI0Manager::writeRed2(bool on){
    setLedBit(2, on);
}
void SPI0Manager::writeRed3(bool on){
    setLedBit(3, on);
}
void SPI0Manager::writeYellow(bool on){
    setLedBit(4, on);
}
void SPI0Manager::writeBlue(bool on){
    setLedBit(5, on);
}

void SPI0Manager::setLedBit(uint8_t ledIndex, bool on) {
    if (ledIndex > 5) return;            // only 0-5

    uint8_t mask = (1 << ledIndex);     // shift one bit to the led position

    if (on) {
        _ledStates |= mask;              // |= does bitwise OR to set bit, keep the ones in both bytes
    } else {
        _ledStates &= ~mask;             // &= does bitwise AND with mask inverted, only the bit at ledIndex is changed to 0
    }

    writeLeds(_ledStates);               // write all 8 bits
}

void SPI0Manager::writeLeds(uint8_t ledStates) {
    _ledStates = ledStates;
    selectDevice(UI_2_LEDS_AND_SWITCHES);
    _ledsAndSwitches.write8(1, ledStates);
    deselectDevice();
}

void SPI0Manager::setScreenRES(bool on){
    // GPB1 pin of main valve MCP23S17
    // update state and call write
    // flip or not flip the second bit
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_RES) : (_mainValveState & ~MAIN_VALVE_RES);
    writeMainValve(_mainValveState);  
}

void SPI0Manager::setScreenBLK(bool on){
    // GPB2 pin of main valve MCP23S17
    // update state and call write
    // flip or not flip the third bit
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_BLK) : (_mainValveState & ~MAIN_VALVE_BLK);
    writeMainValve(_mainValveState);
}

void SPI0Manager::setMainValve(bool on){
    // GPB0 pin of main valve MCP23S17
    // update state and call write
    // flip or not flip the first bit
    _mainValveState = on ? (_mainValveState | MAIN_VALVE_VALVE) : (_mainValveState & ~MAIN_VALVE_VALVE);
    writeMainValve(_mainValveState);
}

void SPI0Manager::writeMainValve(uint16_t state){
    selectDevice(MAINVALVE);
    _mainValve.write16(state);
    //_mainValve.write8(0, stateA); // open main valve
    //_mainValve.write8(1, stateB); // open main valve
    deselectDevice();
}

uint16_t SPI0Manager::readPotmeters(bool id){
    // Read MCP3004. Uses 10-bit, so value 0..1023 (for 0..3.3V)

    selectDevice(ADC_POTMETERS);
    delayMicroseconds(10);
    uint16_t value = 1023 - _adcPotmeters.analogRead(id);  // accidentally inverted; in next version this will be corrected
    deselectDevice();
    return value;
}

// void SPI0Manager::updateScreen(GFXcanvas1 * canvas){
//     if (!canvas) return;
//     selectDevice(SCREEN);           // CS LOW
//     _screen.drawBitmap(0, 0, canvas->getBuffer(), canvas->width(), canvas->height(), 0xFFFF, 0x0000);
//     deselectDevice();               // CS HIGH
// }

uint16_t SPI0Manager::readExternalDevice(deviceNameSPI0 device){
    // uint16_t value;
    // switch(device){
    //     case EXTERNAL_MCP_1:
    //         selectDevice(EXTERNAL_MCP_1);
    //         value =  _externalMCP1.read16();
    //         deselectDevice();
    //         break;
    //     // add more external devices here
    //     default:
    //         value = 0;
    //         break;
    // }
    // return value;
    return -1;
}

void SPI0Manager::writeExternalDevice(deviceNameSPI0 device, uint16_t data){
    // switch(device){
    //     case EXTERNAL_MCP_1:
    //         selectDevice(EXTERNAL_MCP_1);
    //         _externalMCP1.write16(data);
    //         deselectDevice();
    //         break;
    //     // add more external devices here
    //     default:
    //         break;
    // }
}