/**
 * @file neopixel.h
 * @brief NeoPixel 3x3 LED Grid Module for Atech
 *
 * 3x3 grid (9 LEDs). Drives both Line A (WS2812B / RGB) and Line B
 * (SK6812 / RGBW) every frame so the same driver works for both module
 * revisions: the unused line is a no-op on whichever module is plugged in.
 *
 * Brightness is clamped to MAX_BRIGHTNESS (20% of raw 0-255) to keep the
 * RGBW variant comfortable to look at.
 */

#ifndef NEOPIXEL_MODULE_H
#define NEOPIXEL_MODULE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class NeoPixelGrid {
public:
    static const uint8_t GRID_SIZE = 3;
    static const uint8_t NUM_LEDS = 9;
    static const uint8_t MAX_BRIGHTNESS = 51;

    NeoPixelGrid(int dataPinA, int dataPinB, uint8_t numLeds = NUM_LEDS);

    void begin();
    void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    void setPixelXY(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b);
    void setAll(uint8_t r, uint8_t g, uint8_t b);
    void setRow(uint8_t row, uint8_t r, uint8_t g, uint8_t b);
    void setColumn(uint8_t col, uint8_t r, uint8_t g, uint8_t b);
    void clear();
    void show();
    void setBrightness(uint8_t brightness);
    uint8_t getBrightness() const;
    uint32_t getPixelColor(uint8_t index) const;
    uint8_t xyToIndex(uint8_t row, uint8_t col) const { return row * GRID_SIZE + col; }

private:
    Adafruit_NeoPixel _stripA;
    Adafruit_NeoPixel _stripB;
    int _dataPinA;
    int _dataPinB;
    uint8_t _numLeds;
};

#endif
