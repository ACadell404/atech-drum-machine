/**
 * @file neopixel.cpp
 * @brief NeoPixel 3x3 LED Grid Implementation
 */

#include "neopixel.h"

NeoPixelGrid::NeoPixelGrid(int dataPinA, int dataPinB, uint8_t numLeds)
    : _stripA(numLeds, dataPinA, NEO_GRB + NEO_KHZ800)
    , _stripB(numLeds, dataPinB, NEO_GRBW + NEO_KHZ800)
    , _dataPinA(dataPinA)
    , _dataPinB(dataPinB)
    , _numLeds(numLeds)
{
}

void NeoPixelGrid::begin() {
    _stripA.begin();
    _stripB.begin();
    _stripA.setBrightness(MAX_BRIGHTNESS);
    _stripB.setBrightness(MAX_BRIGHTNESS);
    clear();
    show();
}

void NeoPixelGrid::setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < _numLeds) {
        _stripA.setPixelColor(index, _stripA.Color(r, g, b));
        _stripB.setPixelColor(index, _stripB.Color(r, g, b, 0));
    }
}

void NeoPixelGrid::setPixelXY(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b) {
    if (row < GRID_SIZE && col < GRID_SIZE) {
        setPixel(xyToIndex(row, col), r, g, b);
    }
}

void NeoPixelGrid::setAll(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t colorA = _stripA.Color(r, g, b);
    uint32_t colorB = _stripB.Color(r, g, b, 0);
    for (uint8_t i = 0; i < _numLeds; i++) {
        _stripA.setPixelColor(i, colorA);
        _stripB.setPixelColor(i, colorB);
    }
}

void NeoPixelGrid::setRow(uint8_t row, uint8_t r, uint8_t g, uint8_t b) {
    if (row < GRID_SIZE) {
        uint32_t colorA = _stripA.Color(r, g, b);
        uint32_t colorB = _stripB.Color(r, g, b, 0);
        for (uint8_t col = 0; col < GRID_SIZE; col++) {
            uint8_t i = xyToIndex(row, col);
            _stripA.setPixelColor(i, colorA);
            _stripB.setPixelColor(i, colorB);
        }
    }
}

void NeoPixelGrid::setColumn(uint8_t col, uint8_t r, uint8_t g, uint8_t b) {
    if (col < GRID_SIZE) {
        uint32_t colorA = _stripA.Color(r, g, b);
        uint32_t colorB = _stripB.Color(r, g, b, 0);
        for (uint8_t row = 0; row < GRID_SIZE; row++) {
            uint8_t i = xyToIndex(row, col);
            _stripA.setPixelColor(i, colorA);
            _stripB.setPixelColor(i, colorB);
        }
    }
}

void NeoPixelGrid::clear() {
    _stripA.clear();
    _stripB.clear();
}

void NeoPixelGrid::show() {
    _stripA.show();
    _stripB.show();
}

void NeoPixelGrid::setBrightness(uint8_t brightness) {
    uint8_t clamped = brightness > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : brightness;
    if (_stripA.getBrightness() == clamped) return;
    _stripA.setBrightness(clamped);
    _stripB.setBrightness(clamped);
}

uint8_t NeoPixelGrid::getBrightness() const {
    return _stripA.getBrightness();
}

uint32_t NeoPixelGrid::getPixelColor(uint8_t index) const {
    if (index >= _numLeds) return 0;
    return _stripA.getPixelColor(index);
}
