/**
 * @file led_strip.h
 * @brief Addressable LED strip driver (WS2811 / WS2812 family) for Atech
 *
 * Drives the data line of an external addressable strip — e.g. a 24 V strip
 * whose LEDs are powered by their own supply. Only the data line (Line A)
 * and GND connect to the Atech board; the strip's +V never does.
 *
 * Built-in effects render in update() at ~50 fps without blocking. The pixel
 * count, colour order and data speed are unknown for salvaged strips, so they
 * can be changed at runtime and are saved to flash (NVS) so they survive a
 * reboot. Two test effects help find them:
 *   test_colors  whole strip cycles pure red -> green -> blue (reveals colour order)
 *   test_count   pixel 0 red, every 10th white, last configured pixel green
 *
 * Wiring:
 *   Port Line A --[330 ohm]--> strip DATA
 *   Board GND   ------------> strip GND (shared with the strip's power supply)
 */

#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>

class Adafruit_NeoPixel;

class LedStrip {
public:
    static constexpr int MAX_PIXELS = 600;
    static constexpr int DEFAULT_PIXELS = 60;

    enum Effect {
        OFF = 0, SOLID, RAINBOW, CHASE, BREATHE, SPARKLE, CUSTOM,
        TEST_COLORS, TEST_COUNT, NUM_EFFECTS
    };

    explicit LedStrip(int dataPin);

    /** Load saved config from flash and start the driver. */
    void begin();
    /** Render the current effect. Call every loop(); rate-limited internally. */
    void update();

    // ---- effects & look -----------------------------------------------
    void setEffect(Effect e);
    Effect getEffect() const { return _effect; }
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t b);                 // 0..255 (default 80)
    uint8_t getBrightness() const { return _brightness; }
    void setSpeed(float s);                        // effect speed multiplier, 0.1..5
    /** One-shot flash over the current effect, fading out (e.g. on a drum hit). */
    void flash(uint8_t r, uint8_t g, uint8_t b, int decayMs = 250);

    // ---- direct pixel control (switches to the CUSTOM effect) ---------
    void setPixel(int i, uint8_t r, uint8_t g, uint8_t b);
    void fillRange(int from, int to, uint8_t r, uint8_t g, uint8_t b);

    // ---- strip configuration (saved to flash) --------------------------
    void setCount(int n);
    int getCount() const { return _count; }
    /** Colour order as sent on the wire: "RGB", "GRB", "BRG", ... */
    bool setOrder(const char* order);
    const char* getOrder() const;
    /** Data rate: 800 (WS2812 / most WS2811) or 400 kHz (some older WS2811). */
    bool setKhz(int khz);
    int getKhz() const { return _khz; }

    // ---- host control over USB serial ---------------------------------
    /** Read `{"action":"<prefix>_<cmd>","value":"..."}` lines and apply them. */
    void pollSerial(const char* prefix);
    bool handleCommand(const char* cmd, const char* value);
    void emitState(const char* prefix);

    // ---- helpers -------------------------------------------------------
    static const char* effectName(Effect e);
    static int parseEffect(const char* s);
    /** "#ff8800", "ff8800", "255,136,0" or a name ("red", "warm", ...). */
    static bool parseColor(const char* s, uint8_t& r, uint8_t& g, uint8_t& b);

private:
    int _pin;
    Adafruit_NeoPixel* _strip;
    uint8_t* _buf;              // unscaled RGB frame, 3 bytes per pixel
    int _count;
    uint8_t _orderIdx;
    int _khz;

    Effect _effect;
    uint8_t _r, _g, _b;
    uint8_t _brightness;
    float _speed;

    float _flashLevel, _flashK;
    uint8_t _fr, _fg, _fb;

    unsigned long _lastFrameMs;
    unsigned long _lastFlashMs;
    float _phase;               // effect time, advanced by speed
    int _testPhase;

    char _rx[160];
    int _rxLen;
    const char* _prefix;

    void _render(float dt);
    void _show();
    void _applyType();
    void _allocBuffer();
    void _saveConfig();
    void _loadConfig();
    void _emitTestPhase(const char* what);
};

#endif // LED_STRIP_H
