/**
 * @file rotary_encoder.h
 * @brief Rotary Encoder Knob Module for Athera
 *
 * 18-step rotary encoder with push-button switch and 12-LED NeoPixel
 * indicator ring. Uses interrupts for reliable step detection at any speed.
 * Built-in acceleration: slow turns = fine control, fast turns = big jumps.
 *
 * The indicator ring shows the knob's current position with a glow/trail
 * effect: primary LED at full brightness, neighbors fading out.
 * Updates automatically each call to update().
 *
 * Double module spanning two adjacent ports:
 * - Port 1 Line A (pin):   CLK (clock)
 * - Port 1 Line B (pin_b): DT  (data)
 * - Port 2 Line A (pin_c): SW  (button, active low)
 * - Port 2 Line B (pin_d): NeoPixel ring data (12 WS2812B LEDs)
 *
 * LED chain layout (12 LEDs total, but backwards-compatible with 6-LED knobs):
 *   Chain indices 0-5 are the original 6 LEDs at the cardinal/intercardinal
 *   positions. Chain indices 6-11 are the new in-between LEDs.
 *   On older 6-LED hardware, chain indices 6-11 are no-ops (data goes nowhere).
 *
 *   Around the ring (counter-clockwise from 12 o'clock):
 *     physical pos:  0   1   2   3   4   5   6   7   8   9  10  11
 *     chain idx:     0   6   1   7   2   8   3   9   4  10   5  11
 */

#ifndef ROTARY_ENCODER_MODULE_H
#define ROTARY_ENCODER_MODULE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class RotaryEncoder {
public:
    static const uint8_t RING_LEDS = 12;
    static const int32_t DETENTS_PER_REV = 18;

    /**
     * @brief Construct rotary encoder
     * @param pinClk  GPIO for clock line (Line A1)
     * @param pinDt   GPIO for data line (Line B1)
     * @param pinSw   GPIO for button switch (Line A2), -1 = no button
     * @param pinRing GPIO for NeoPixel ring data (Line B2), -1 = no ring
     */
    RotaryEncoder(int pinClk, int pinDt, int pinSw = -1, int pinRing = -1);

    /** @brief Initialize encoder, button, and ring; attach interrupts */
    void begin();

    /**
     * @brief Update button state and ring indicator (call once per loop)
     * Encoder position is updated via interrupts automatically.
     * Ring LEDs are refreshed here based on current position.
     */
    void update();

    /** @brief Get cumulative position count (with acceleration applied) */
    int32_t getPosition();

    /** @brief Set position to a specific value */
    void setPosition(int32_t pos);

    /** @brief Reset position to zero */
    void resetPosition();

    /**
     * @brief Get direction of last rotation
     * @return 1 = clockwise, -1 = counter-clockwise, 0 = no movement
     */
    int getDirection();

    /** @brief True if knob rotated clockwise since last check */
    bool wasRotatedCW();

    /** @brief True if knob rotated counter-clockwise since last check */
    bool wasRotatedCCW();

    /** @brief Check if button is currently pressed */
    bool isPressed();

    /** @brief Check if button was just pressed (edge detection, clears after read) */
    bool wasPressed();

    /**
     * @brief Set acceleration parameters
     * @param enabled  Enable/disable acceleration (default: enabled)
     * @param maxMultiplier  Maximum step multiplier when spinning fast (default: 5)
     */
    void setAcceleration(bool enabled, int maxMultiplier = 5);

    // --- Ring indicator methods ---

    /**
     * @brief Enable or disable the ring indicator
     * @param enabled  true = ring shows position, false = ring off
     */
    void enableRing(bool enabled);

    /**
     * @brief Set ring indicator color
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setRingColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set ring global brightness
     * @param brightness 0-255
     */
    void setRingBrightness(uint8_t brightness);

    /**
     * @brief Manually override ring position (decouples from knob)
     * @param pos  Ring position as float (0.0 to 12.0, wraps)
     *             Position is measured around the physical ring (counter-clockwise
     *             from 12 o'clock). The driver handles the chain interleaving.
     *             Pass NAN or negative to re-couple to knob position.
     */
    void setRingPosition(float pos);

private:
    int _pinClk;
    int _pinDt;
    int _pinSw;
    int _pinRing;

    volatile int32_t _position;
    volatile int32_t _rawPosition;  // Non-accelerated position for ring indicator
    volatile int _lastDirection;
    volatile bool _cwFlag;
    volatile bool _ccwFlag;

    // Acceleration
    bool _accelEnabled;
    int _accelMaxMultiplier;
    volatile unsigned long _lastStepTime;

    // Button
    bool _lastBtnState;
    bool _pressedFlag;

    // Ring
    Adafruit_NeoPixel* _ring;
    bool _ringEnabled;
    uint8_t _ringR, _ringG, _ringB;
    float _ringOverridePos;  // negative = follow knob
    int32_t _lastRingPosition; // last position used for ring update

    // Background ring renderer — wakes from ISR for sub-millisecond response
    TaskHandle_t _ringTask;

    void _updateRing();
    void _renderRing(float pos);
    static void _ringTaskTrampoline(void* param);
    void _ringTaskLoop();
    void _wakeRingTask();

    // ISR
    static void IRAM_ATTR _isrHandler(void* arg);
    void IRAM_ATTR _handleInterrupt();
    int IRAM_ATTR _calcAccelMultiplier(unsigned long elapsed);
};

#endif // ROTARY_ENCODER_MODULE_H
