/**
 * @file tw_light.h
 * @brief Tunable-white (warm + cold) 24 V LED light, PWM-dimmed through MOSFETs.
 *
 * Built for a COB "TW" profile: +24 V common anode, separate warm and cold
 * cathodes. Each cathode is switched to GND by a logic-level N-MOSFET whose
 * gate is driven from one line of an Atech port:
 *
 *   Line A (data 1) -> MOSFET gate -> WARM channel
 *   Line B (data 2) -> MOSFET gate -> COLD channel
 *   24 V PSU minus and Atech GND tied together. 24 V never touches the board.
 *
 * PWM runs at ~19.5 kHz (inaudible, no flicker on camera) with 12-bit
 * resolution and a gamma curve, so fades look smooth.
 *
 * Music: hit() fires a flash that decays back to the base look; setEnvelope()
 * lets a loudness signal (microphone or host) drive brightness in MUSIC mode.
 */

#ifndef TW_LIGHT_H
#define TW_LIGHT_H

#include <Arduino.h>

class TwLight {
public:
    enum Mode { OFF = 0, STATIC, BREATHE, STROBE, MUSIC, NUM_MODES };

    static constexpr int MIN_KELVIN = 2200;
    static constexpr int MAX_KELVIN = 6500;

    TwLight(int pinWarm, int pinCold);

    void begin();
    /** Render the current look. Call every loop(); rate-limited internally. */
    void update();

    // ---- base look -------------------------------------------------------
    void setMode(Mode m);
    Mode getMode() const { return _mode; }
    void setBrightness(float level);          // 0..1
    float getBrightness() const { return _brightness; }
    /** Colour temperature, 0 = warmest .. 1 = coldest. */
    void setTemperature(float t);
    float getTemperature() const { return _temp; }
    void setKelvin(int k);
    int getKelvin() const;
    void setSpeed(float s);                   // breathe / strobe rate multiplier
    void setStrobeHz(float hz);

    // ---- music -------------------------------------------------------------
    /** Flash: jump to `intensity` at temperature `temp`, fade out over decayMs. */
    void hit(float intensity, float temp, int decayMs);
    /** Loudness 0..1 for MUSIC mode (fast attack, slower release applied here). */
    void setEnvelope(float level);
    /** Whether the firmware should fire hit() on drum hits (toggled with `<prefix>_beat`). */
    bool beatSync() const { return _beatSync; }
    void setBeatSync(bool on) { _beatSync = on; }

    // ---- host control over USB serial --------------------------------------
    void pollSerial(const char* prefix);
    bool handleCommand(const char* cmd, const char* value);
    void emitState(const char* prefix);
    const char* busPrefix() const { return _prefix; }

    static const char* modeName(Mode m);
    static int parseMode(const char* s);

private:
    int _pinWarm, _pinCold;
    bool _ready;

    Mode _mode;
    float _brightness;
    float _temp;
    float _speed;
    float _strobeHz;
    bool _beatSync;

    float _flash, _flashTemp, _flashTau;
    float _envTarget, _env;

    unsigned long _lastUs;
    float _phase;
    const char* _prefix;

    void _write(float warm, float cold);
    static void _mix(float level, float temp, float& warm, float& cold);
};

#endif // TW_LIGHT_H
