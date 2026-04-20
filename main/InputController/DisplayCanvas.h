#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class DisplayCanvas {
public:
    DisplayCanvas(int width, int height, uint16_t* buffer);

    int width() const { return _width; }
    int height() const { return _height; }
    uint16_t* getBuffer() { return _buffer; }
    const uint16_t* getBuffer() const { return _buffer; }

    void fillScreen(uint16_t color);
    void fillRect(int x, int y, int w, int h, uint16_t color);
    void setTextColor(uint16_t color);
    void setTextSize(uint8_t size);
    void setCursor(int x, int y);

    void print(const char* text);
    void print(const std::string& text);
    void print(char c);
    void print(int value);
    void print(unsigned value);
    void print(float value, int decimals = 2);
    void printf(const char* fmt, ...);

private:
    void drawChar(int x, int y, char c, uint16_t color, uint8_t size);
    void drawPixel(int x, int y, uint16_t color);
    const uint8_t* glyphFor(char c) const;

private:
    int _width = 0;
    int _height = 0;
    uint16_t* _buffer = nullptr;
    int _cursorX = 0;
    int _cursorY = 0;
    uint16_t _textColor = 0xFFFF;
    uint8_t _textSize = 1;
};
