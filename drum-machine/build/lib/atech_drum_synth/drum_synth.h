/**
 * @file drum_synth.h
 * @brief 6-voice percussion synthesizer + 16-step sequencer on the I2S speaker
 *
 * Drives the same MAX98357A speaker hardware as the stock `speaker` module, but
 * renders drums instead of sine tones: every voice is synthesized per-sample
 * (pitch-swept sines, filtered noise, 808-style metallic oscillators).
 *
 * The audio + sequencer clock run in one FreeRTOS task on Core 0, so timing is
 * sample-accurate and never jitters when the main loop is busy (e.g. pushing
 * pixels to a display).
 *
 * Voices / tracks:
 *   0 KICK   1 SNARE   2 CLAP   3 CLOSED HAT   4 OPEN HAT   5 TOM
 *
 * Connector (double module, same wiring as `speaker`):
 *   Primary port   Line A: LRCLK   Line B: BCLK
 *   Secondary port Line B: DIN
 */

#ifndef DRUM_SYNTH_H
#define DRUM_SYNTH_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class DrumSynth {
public:
    static constexpr int NUM_TRACKS = 6;
    static constexpr int NUM_STEPS = 16;
    static constexpr int SAMPLE_RATE = 32000;
    static constexpr int MIN_BPM = 50;
    static constexpr int MAX_BPM = 220;
    static constexpr int MIN_TUNE = -12;
    static constexpr int MAX_TUNE = 12;
    static constexpr int MAX_LEVEL = 100;

    enum Track { KICK = 0, SNARE, CLAP, CLOSED_HAT, OPEN_HAT, TOM };

    DrumSynth(int bclkPin, int lrcPin, int doutPin);

    /** Install the I2S driver and start the audio/sequencer task. */
    void begin();

    // ---- live playing -------------------------------------------------
    /** Fire a drum right now (thread-safe, returns immediately). */
    void trigger(int track, float velocity = 1.0f);

    // ---- transport ----------------------------------------------------
    void play();
    void stop();
    void togglePlay();
    bool isPlaying() const { return _playing; }
    void setTempo(int bpm);
    int getTempo() const { return _bpm; }

    // ---- pattern ------------------------------------------------------
    bool getStep(int track, int step) const;
    void setStep(int track, int step, bool on);
    void toggleStep(int track, int step);
    uint16_t getPattern(int track) const;
    void setPattern(int track, uint16_t bits);
    /** Set a track from a 16-char string: 'x', 'X' or '1' = hit, anything else = rest. */
    void setPatternString(int track, const char* s);
    void clearTrack(int track);
    void clearAll();
    void loadDemoBeat();
    /** Bumped on every pattern edit — lets a UI notice changes made elsewhere. */
    uint32_t patternVersion() const { return _patternVersion; }

    // ---- sound shaping ------------------------------------------------
    /** Per-drum velocity 0..100 %, applied to sequenced and live hits. 0 mutes the drum. */
    void setLevel(int track, int percent);
    int getLevel(int track) const;
    void setTune(int track, int semitones);
    int getTune(int track) const;
    void setVolume(float v);
    float getVolume() const { return _volume; }

    // ---- playhead (for UIs) -------------------------------------------
    /** Step the playhead is on, or -1 when stopped. */
    int currentStep() const { return _curStep; }
    /** True once after every playhead move (or stop). Reading clears it. */
    bool stepAdvanced();
    /** Bitmask of tracks that fired on the most recent step. */
    uint8_t lastHits() const { return _lastHits; }

    // ---- host control over USB serial ---------------------------------
    /**
     * Read `{"action":"<prefix>_<cmd>","value":"..."}` lines sent by
     * atech.Board.send() and apply them. Also emits events: `<prefix>_step`
     * on every playhead move, `<prefix>_playing` / `<prefix>_tempo` on change,
     * and a full `<prefix>_state` snapshot whenever the pattern, tuning or
     * volume changes (from anywhere — knobs or host).
     */
    void pollSerial(const char* prefix);
    /** Apply one command (without the prefix). Returns false if unknown. */
    bool handleCommand(const char* cmd, const char* value);

    // ---- helpers ------------------------------------------------------
    static const char* trackName(int track);
    /** Parse "3", "kick", "snare", "clap", "chh"/"closed", "ohh"/"open", "tom". -1 if unknown. */
    static int parseTrack(const char* s);
    static uint16_t trackColor565(int track);
    static void trackColorRGB(int track, uint8_t& r, uint8_t& g, uint8_t& b);

    // Background task body (public for the FreeRTOS trampoline — don't call).
    void _audioTaskLoop();

private:
    struct Voice {
        bool active;
        float vel;
        float amp, ampK;          // main envelope + per-sample decay multiplier
        float amp2, amp2K;        // secondary envelope (noise / click layer)
        float phase, phase2;      // sine phases, 0..1
        float freq, freqBase;     // current + resting pitch (Hz)
        float pitchEnv, pitchK;   // pitch-sweep envelope
        float oscPh[6];           // metallic oscillator phases (hats)
        float hpX, hpY;           // one-pole high-pass state
        float hp2X, hp2Y;         // second high-pass stage
        float svfLow, svfBand;    // state-variable band-pass (clap)
        uint32_t age;             // samples since trigger
    };

    int _bclkPin, _lrcPin, _doutPin;
    bool _initialized;

    Voice _voices[NUM_TRACKS];
    volatile uint16_t _pattern[NUM_TRACKS];
    volatile int8_t _tune[NUM_TRACKS];
    volatile uint8_t _level[NUM_TRACKS];
    volatile uint32_t _patternVersion;

    volatile int _bpm;
    volatile float _volume;
    volatile bool _playing;
    volatile bool _playReq, _stopReq;
    volatile int _curStep;
    volatile bool _stepFlag;
    volatile uint8_t _lastHits;

    volatile uint8_t _pendingMask;       // triggers queued from other cores
    float _pendingVel[NUM_TRACKS];
    portMUX_TYPE _mux;

    // Audio-task-only state
    float _stepCounter;
    uint32_t _noise;

    // Change tracking for events
    bool _reportedPlaying;
    int _reportedBpm;
    int _reportedStep;
    uint32_t _reportedVersion;
    int8_t _reportedTune[NUM_TRACKS];
    uint8_t _reportedLevel[NUM_TRACKS];
    float _reportedVolume;
    unsigned long _stateChangedMs;
    bool _stateDirty;

    void _emitState(const char* prefix);

    TaskHandle_t _task;

    void _startVoice(int track, float vel);
    float _renderVoice(int track);
    inline float _rand();   // white noise, -1..1
    void _bumpVersion() { _patternVersion = _patternVersion + 1; }
};

// Header-only UI helpers (screen + debounced buttons). Templated on the display
// class so this library doesn't depend on any particular display driver.
#include "drum_ui.h"

#endif // DRUM_SYNTH_H
