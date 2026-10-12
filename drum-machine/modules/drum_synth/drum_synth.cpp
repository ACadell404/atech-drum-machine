/**
 * @file drum_synth.cpp
 * @brief Percussion synth voices, sequencer clock and serial control.
 */

#include "drum_synth.h"
#include "atech_bus.h"
#include <driver/i2s.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

// ------------------------------------------------------------ constants

static const i2s_port_t DRUM_I2S_PORT = I2S_NUM_0;
static const int BLOCK_FRAMES = 64;
static const float SR = (float)DrumSynth::SAMPLE_RATE;

// Sine lookup table (one cycle + guard point for interpolation)
static const int SINE_SIZE = 512;
static float s_sine[SINE_SIZE + 1];

// Filter coefficients, computed once in begin()
static float s_hpHat;     // one-pole high-pass @ 7 kHz (hats)
static float s_hpSnare;   // one-pole high-pass @ 1.5 kHz (snare noise)
static float s_svfClapF;  // state-variable band-pass centre @ 1.1 kHz (clap)
static const float SVF_CLAP_DAMP = 0.6f;

// Classic TR-808 cymbal/hat oscillator frequencies (Hz)
static const float HAT_FREQS[6] = { 205.3f, 304.4f, 369.6f, 522.7f, 540.0f, 800.0f };

// Per-track mix levels
static const float MIX[DrumSynth::NUM_TRACKS] = { 0.95f, 0.55f, 0.55f, 0.32f, 0.32f, 0.65f };

// Clap: three short bursts then a tail
static const uint32_t CLAP_BURST = (uint32_t)(0.011f * DrumSynth::SAMPLE_RATE);

static inline float decayK(float tauSeconds) {
    return expf(-1.0f / (tauSeconds * SR));
}

static inline float onePoleHpA(float fc) {
    float rc = 1.0f / (2.0f * (float)M_PI * fc);
    float dt = 1.0f / SR;
    return rc / (rc + dt);
}

static inline float sineLookup(float phase) {
    float idx = phase * SINE_SIZE;
    int i = (int)idx;
    float frac = idx - i;
    i &= (SINE_SIZE - 1);
    return s_sine[i] + (s_sine[i + 1] - s_sine[i]) * frac;
}

static inline float wrap01(float p) {
    return p >= 1.0f ? p - (int)p : p;
}

// Cubic soft clipper: transparent near 0, smooth ceiling at ±1
static inline float softClip(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x * (1.5f - 0.5f * x * x);
}

// Gentle saturation for the kick (adds harmonics so it's audible on small speakers)
static inline float saturate(float x) {
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

static void audioTaskTrampoline(void* param) {
    ((DrumSynth*)param)->_audioTaskLoop();
}

// ------------------------------------------------------------ lifecycle

DrumSynth::DrumSynth(int bclkPin, int lrcPin, int doutPin)
    : _bclkPin(bclkPin)
    , _lrcPin(lrcPin)
    , _doutPin(doutPin)
    , _initialized(false)
    , _patternVersion(0)
    , _bpm(120)
    , _volume(0.5f)
    , _playing(false)
    , _playReq(false)
    , _stopReq(false)
    , _curStep(-1)
    , _stepFlag(false)
    , _lastHits(0)
    , _pendingMask(0)
    , _stepCounter(0)
    , _noise(0x12345678u)
    , _reportedPlaying(false)
    , _reportedBpm(120)
    , _reportedStep(-1)
    , _reportedVersion(0)
    , _reportedVolume(0.5f)
    , _stateChangedMs(0)
    , _stateDirty(false)
    , _task(nullptr)
{
    _mux = portMUX_INITIALIZER_UNLOCKED;
    memset(_voices, 0, sizeof(_voices));
    for (int t = 0; t < NUM_TRACKS; t++) {
        _pattern[t] = 0;
        _tune[t] = 0;
        _reportedTune[t] = 0;
        _level[t] = MAX_LEVEL;
        _reportedLevel[t] = MAX_LEVEL;
        _pendingVel[t] = 1.0f;
    }
    loadDemoBeat();
    _reportedVersion = _patternVersion;
}

void DrumSynth::begin() {
    for (int i = 0; i <= SINE_SIZE; i++) {
        s_sine[i] = sinf(2.0f * (float)M_PI * i / SINE_SIZE);
    }
    s_hpHat = onePoleHpA(7000.0f);
    s_hpSnare = onePoleHpA(1500.0f);
    s_svfClapF = 2.0f * sinf((float)M_PI * 1100.0f / SR);

    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = (uint32_t)SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 128,     // 4 x 128 frames = 16 ms of buffering
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };
    i2s_pin_config_t pins = {
        .bck_io_num = _bclkPin,
        .ws_io_num = _lrcPin,
        .data_out_num = _doutPin,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };

    esp_err_t err = i2s_driver_install(DRUM_I2S_PORT, &cfg, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("DrumSynth: I2S install failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = i2s_set_pin(DRUM_I2S_PORT, &pins);
    if (err != ESP_OK) {
        Serial.printf("DrumSynth: I2S pin config failed: %s\n", esp_err_to_name(err));
        return;
    }
    _initialized = true;

    // Core 0, high priority: audio must never starve. The main loop runs on Core 1.
    xTaskCreatePinnedToCore(audioTaskTrampoline, "DrumSynth", 6144, this,
                            configMAX_PRIORITIES - 2, &_task, 0);
}

// ------------------------------------------------------------ public API

void DrumSynth::trigger(int track, float velocity) {
    if (track < 0 || track >= NUM_TRACKS) return;
    portENTER_CRITICAL(&_mux);
    _pendingVel[track] = constrain(velocity, 0.0f, 1.0f);
    _pendingMask |= (1 << track);
    portEXIT_CRITICAL(&_mux);
}

void DrumSynth::play() { _playReq = true; }
void DrumSynth::stop() { _stopReq = true; }
void DrumSynth::togglePlay() { if (_playing) stop(); else play(); }

void DrumSynth::setTempo(int bpm) {
    _bpm = constrain(bpm, MIN_BPM, MAX_BPM);
}

bool DrumSynth::getStep(int track, int step) const {
    if (track < 0 || track >= NUM_TRACKS || step < 0 || step >= NUM_STEPS) return false;
    return (_pattern[track] >> step) & 1;
}

void DrumSynth::setStep(int track, int step, bool on) {
    if (track < 0 || track >= NUM_TRACKS || step < 0 || step >= NUM_STEPS) return;
    uint16_t bits = _pattern[track];
    bits = on ? (bits | (1u << step)) : (bits & ~(1u << step));
    _pattern[track] = bits;
    _bumpVersion();
}

void DrumSynth::toggleStep(int track, int step) {
    setStep(track, step, !getStep(track, step));
}

uint16_t DrumSynth::getPattern(int track) const {
    return (track >= 0 && track < NUM_TRACKS) ? _pattern[track] : 0;
}

void DrumSynth::setPattern(int track, uint16_t bits) {
    if (track < 0 || track >= NUM_TRACKS) return;
    _pattern[track] = bits;
    _bumpVersion();
}

void DrumSynth::setPatternString(int track, const char* s) {
    uint16_t bits = 0;
    for (int i = 0; i < NUM_STEPS && s && s[i]; i++) {
        char c = s[i];
        if (c == 'x' || c == 'X' || c == '1') bits |= (1u << i);
    }
    setPattern(track, bits);
}

void DrumSynth::clearTrack(int track) { setPattern(track, 0); }

void DrumSynth::clearAll() {
    for (int t = 0; t < NUM_TRACKS; t++) _pattern[t] = 0;
    _bumpVersion();
}

void DrumSynth::loadDemoBeat() {
    static const char* DEMO[NUM_TRACKS] = {
        "x.....x...x.....",   // kick
        "....x.......x...",   // snare
        "............x...",   // clap
        "x.x.x.x.x.x.x...",   // closed hat
        "..............x.",   // open hat
        ".......x.......x",   // tom
    };
    for (int t = 0; t < NUM_TRACKS; t++) {
        uint16_t bits = 0;
        for (int i = 0; i < NUM_STEPS; i++) {
            if (DEMO[t][i] == 'x') bits |= (1u << i);
        }
        _pattern[t] = bits;
    }
    _bumpVersion();
}

void DrumSynth::setLevel(int track, int percent) {
    if (track < 0 || track >= NUM_TRACKS) return;
    _level[track] = (uint8_t)constrain(percent, 0, MAX_LEVEL);
}

int DrumSynth::getLevel(int track) const {
    return (track >= 0 && track < NUM_TRACKS) ? _level[track] : 0;
}

void DrumSynth::setTune(int track, int semitones) {
    if (track < 0 || track >= NUM_TRACKS) return;
    _tune[track] = (int8_t)constrain(semitones, MIN_TUNE, MAX_TUNE);
}

int DrumSynth::getTune(int track) const {
    return (track >= 0 && track < NUM_TRACKS) ? _tune[track] : 0;
}

void DrumSynth::setVolume(float v) {
    _volume = constrain(v, 0.0f, 1.0f);
}

bool DrumSynth::stepAdvanced() {
    if (_stepFlag) {
        _stepFlag = false;
        return true;
    }
    return false;
}

// ------------------------------------------------------------ helpers

const char* DrumSynth::trackName(int track) {
    static const char* NAMES[NUM_TRACKS] = { "KICK", "SNARE", "CLAP", "C.HAT", "O.HAT", "TOM" };
    return (track >= 0 && track < NUM_TRACKS) ? NAMES[track] : "?";
}

int DrumSynth::parseTrack(const char* s) {
    if (!s || !*s) return -1;
    if (isdigit((unsigned char)s[0])) {
        int n = atoi(s);
        return (n >= 0 && n < NUM_TRACKS) ? n : -1;
    }
    char low[12];
    int i = 0;
    for (; s[i] && i < 11; i++) low[i] = tolower((unsigned char)s[i]);
    low[i] = 0;
    if (!strcmp(low, "kick") || !strcmp(low, "bd")) return KICK;
    if (!strcmp(low, "snare") || !strcmp(low, "sd")) return SNARE;
    if (!strcmp(low, "clap") || !strcmp(low, "cp")) return CLAP;
    if (!strcmp(low, "chh") || !strcmp(low, "closed") || !strcmp(low, "hat") ||
        !strcmp(low, "hh") || !strcmp(low, "ch")) return CLOSED_HAT;
    if (!strcmp(low, "ohh") || !strcmp(low, "open") || !strcmp(low, "oh")) return OPEN_HAT;
    if (!strcmp(low, "tom") || !strcmp(low, "lt")) return TOM;
    return -1;
}

void DrumSynth::trackColorRGB(int track, uint8_t& r, uint8_t& g, uint8_t& b) {
    static const uint8_t C[NUM_TRACKS][3] = {
        {255,  60,  40},   // kick  - red
        {255, 200,   0},   // snare - yellow
        {255,  80, 200},   // clap  - pink
        {  0, 220, 255},   // c.hat - cyan
        { 70, 120, 255},   // o.hat - blue
        { 80, 255,  80},   // tom   - green
    };
    if (track < 0 || track >= NUM_TRACKS) { r = g = b = 255; return; }
    r = C[track][0]; g = C[track][1]; b = C[track][2];
}

uint16_t DrumSynth::trackColor565(int track) {
    uint8_t r, g, b;
    trackColorRGB(track, r, g, b);
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ------------------------------------------------------------ voices

inline float DrumSynth::_rand() {
    // xorshift32
    uint32_t x = _noise;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    _noise = x;
    return (int32_t)x * (1.0f / 2147483648.0f);
}

void DrumSynth::_startVoice(int track, float vel) {
    Voice& v = _voices[track];
    float ratio = powf(2.0f, _tune[track] / 12.0f);

    memset(&v, 0, sizeof(Voice));
    v.active = true;
    v.vel = vel;

    switch (track) {
        case KICK:
            v.freqBase = 50.0f * ratio;
            v.pitchEnv = 1.0f;  v.pitchK = decayK(0.030f);
            v.amp = 1.0f;       v.ampK = decayK(0.170f);
            v.amp2 = 0.5f;      v.amp2K = decayK(0.002f);   // attack click
            break;
        case SNARE:
            v.freqBase = 185.0f * ratio;
            v.amp = 0.7f;       v.ampK = decayK(0.045f);    // body
            v.amp2 = 1.0f;      v.amp2K = decayK(0.080f);   // rattle
            break;
        case CLAP:
            v.amp = 1.0f;       v.ampK = decayK(0.004f);
            break;
        case CLOSED_HAT:
            v.freqBase = ratio;
            v.amp = 1.0f;       v.ampK = decayK(0.022f);
            // Choke the open hat, like a real hi-hat pedal
            if (_voices[OPEN_HAT].active) _voices[OPEN_HAT].ampK = decayK(0.004f);
            break;
        case OPEN_HAT:
            v.freqBase = ratio;
            v.amp = 1.0f;       v.ampK = decayK(0.180f);
            break;
        case TOM:
            v.freqBase = 105.0f * ratio;
            v.pitchEnv = 1.0f;  v.pitchK = decayK(0.070f);
            v.amp = 1.0f;       v.ampK = decayK(0.140f);
            v.amp2 = 0.25f;     v.amp2K = decayK(0.008f);
            break;
    }
}

float DrumSynth::_renderVoice(int track) {
    Voice& v = _voices[track];
    float out = 0.0f;

    switch (track) {
        case KICK: {
            v.freq = v.freqBase * (1.0f + 3.5f * v.pitchEnv);
            v.phase = wrap01(v.phase + v.freq / SR);
            float body = saturate(1.6f * sineLookup(v.phase) * v.amp);
            out = body + _rand() * v.amp2 * 0.3f;
            v.pitchEnv *= v.pitchK;
            v.amp *= v.ampK;
            v.amp2 *= v.amp2K;
            break;
        }
        case SNARE: {
            v.phase = wrap01(v.phase + v.freqBase / SR);
            v.phase2 = wrap01(v.phase2 + (v.freqBase * 1.78f) / SR);
            float body = (sineLookup(v.phase) + 0.5f * sineLookup(v.phase2)) * v.amp;
            float n = _rand();
            v.hpY = s_hpSnare * (v.hpY + n - v.hpX);
            v.hpX = n;
            out = 0.6f * body + 0.9f * v.hpY * v.amp2;
            v.amp *= v.ampK;
            v.amp2 *= v.amp2K;
            break;
        }
        case CLAP: {
            // Retrigger the envelope for bursts 2 and 3, then switch to a long tail
            if (v.age == CLAP_BURST || v.age == 2 * CLAP_BURST) {
                v.amp = 0.9f;
            } else if (v.age == 3 * CLAP_BURST) {
                v.amp = 0.8f;
                v.ampK = decayK(0.075f);
            }
            float n = _rand();
            float high = n - v.svfLow - SVF_CLAP_DAMP * v.svfBand;
            v.svfBand += s_svfClapF * high;
            v.svfLow += s_svfClapF * v.svfBand;
            out = 1.8f * v.svfBand * v.amp;
            v.amp *= v.ampK;
            break;
        }
        case CLOSED_HAT:
        case OPEN_HAT: {
            float metal = 0.0f;
            for (int i = 0; i < 6; i++) {
                v.oscPh[i] = wrap01(v.oscPh[i] + HAT_FREQS[i] * v.freqBase / SR);
                metal += v.oscPh[i] < 0.5f ? 1.0f : -1.0f;
            }
            float x = metal * (1.0f / 6.0f) + 0.35f * _rand();
            // Two high-pass stages strip the low end, leaving the shimmer
            v.hpY = s_hpHat * (v.hpY + x - v.hpX);
            v.hpX = x;
            v.hp2Y = s_hpHat * (v.hp2Y + v.hpY - v.hp2X);
            v.hp2X = v.hpY;
            out = 1.6f * v.hp2Y * v.amp;
            v.amp *= v.ampK;
            break;
        }
        case TOM: {
            v.freq = v.freqBase * (1.0f + 0.6f * v.pitchEnv);
            v.phase = wrap01(v.phase + v.freq / SR);
            out = sineLookup(v.phase) * v.amp + _rand() * v.amp2 * 0.3f;
            v.pitchEnv *= v.pitchK;
            v.amp *= v.ampK;
            v.amp2 *= v.amp2K;
            break;
        }
    }

    v.age++;
    // Clap must survive its burst gaps; everything else ends once both envelopes fade
    bool clapInBursts = (track == CLAP && v.age <= 3 * CLAP_BURST);
    if (!clapInBursts && v.amp < 0.0005f && v.amp2 < 0.0005f) {
        v.active = false;
    }
    return out * v.vel;
}

// ------------------------------------------------------------ audio task

void DrumSynth::_audioTaskLoop() {
    int16_t buf[BLOCK_FRAMES * 2];

    while (true) {
        // Transport requests from the main loop / serial
        if (_playReq) {
            _playReq = false;
            _playing = true;
            _curStep = -1;
            _stepCounter = 1e9f;   // fire step 0 on the very next sample
        }
        if (_stopReq) {
            _stopReq = false;
            _playing = false;
            _curStep = -1;
            _lastHits = 0;
            _stepFlag = true;
        }

        // Live triggers queued from the other core
        uint8_t mask;
        float vel[NUM_TRACKS];
        portENTER_CRITICAL(&_mux);
        mask = _pendingMask;
        _pendingMask = 0;
        for (int t = 0; t < NUM_TRACKS; t++) vel[t] = _pendingVel[t];
        portEXIT_CRITICAL(&_mux);
        for (int t = 0; t < NUM_TRACKS; t++) {
            if ((mask & (1 << t)) && _level[t] > 0) _startVoice(t, vel[t] * _level[t] * 0.01f);
        }

        const float samplesPerStep = SR * 60.0f / (_bpm * 4.0f);   // 16th notes
        const float vol = _volume;

        for (int f = 0; f < BLOCK_FRAMES; f++) {
            // Sample-accurate sequencer clock
            if (_playing) {
                _stepCounter += 1.0f;
                if (_stepCounter >= samplesPerStep) {
                    _stepCounter = (_stepCounter > samplesPerStep * 2) ? 0.0f
                                                                       : _stepCounter - samplesPerStep;
                    int step = (_curStep + 1) % NUM_STEPS;
                    uint8_t hits = 0;
                    for (int t = 0; t < NUM_TRACKS; t++) {
                        if (((_pattern[t] >> step) & 1) && _level[t] > 0) {
                            _startVoice(t, _level[t] * 0.01f);
                            hits |= (1 << t);
                        }
                    }
                    _lastHits = hits;
                    _curStep = step;
                    _stepFlag = true;
                }
            }

            float mix = 0.0f;
            for (int t = 0; t < NUM_TRACKS; t++) {
                if (_voices[t].active) mix += MIX[t] * _renderVoice(t);
            }
            int16_t s = (int16_t)(softClip(mix * 0.8f) * vol * 32000.0f);
            buf[f * 2] = s;
            buf[f * 2 + 1] = s;
        }

        size_t written = 0;
        i2s_write(DRUM_I2S_PORT, buf, sizeof(buf), &written, portMAX_DELAY);
    }
}

// ------------------------------------------------------------ serial control

static void emitEvent(const char* prefix, const char* key, const char* type, long value) {
    Serial.printf("{\"type\":\"event\",\"payload\":{\"event_type\":\"%s\",\"key\":\"%s_%s\","
                  "\"value\":%ld,\"source\":\"drum_synth\"}}\n",
                  type, prefix, key, value);
}

static bool busHandler(void* ctx, const char* cmd, const char* value) {
    return static_cast<DrumSynth*>(ctx)->handleCommand(cmd, value);
}

void DrumSynth::pollSerial(const char* prefix) {
    // Commands arrive through the shared router so other modules can share Serial
    atech_bus::subscribe(prefix, busHandler, this);
    atech_bus::poll();

    // State-change events, whether the change came from the host or the hardware
    if (_curStep != _reportedStep) {
        _reportedStep = _curStep;
        emitEvent(prefix, "step", "state", _reportedStep);
    }
    if (_playing != _reportedPlaying) {
        _reportedPlaying = _playing;
        emitEvent(prefix, "playing", "state", _playing ? 1 : 0);
    }
    if (_bpm != _reportedBpm) {
        _reportedBpm = _bpm;
        emitEvent(prefix, "tempo", "state", _bpm);
    }

    // Pattern / tune / volume edits: send one full snapshot once edits go quiet
    bool changed = (_patternVersion != _reportedVersion) || (_volume != _reportedVolume);
    for (int t = 0; t < NUM_TRACKS && !changed; t++) {
        changed = (_tune[t] != _reportedTune[t]) || (_level[t] != _reportedLevel[t]);
    }
    if (changed) {
        _reportedVersion = _patternVersion;
        _reportedVolume = _volume;
        for (int t = 0; t < NUM_TRACKS; t++) {
            _reportedTune[t] = _tune[t];
            _reportedLevel[t] = _level[t];
        }
        _stateDirty = true;
        _stateChangedMs = millis();
    }
    if (_stateDirty && millis() - _stateChangedMs >= 80) {
        _emitState(prefix);
    }
}

void DrumSynth::_emitState(const char* prefix) {
    _stateDirty = false;
    Serial.printf("{\"type\":\"event\",\"payload\":{\"event_type\":\"state\",\"key\":\"%s_state\","
                  "\"value\":{\"bpm\":%d,\"playing\":%d,\"step\":%d,\"volume\":%.2f,\"tracks\":[",
                  prefix, _bpm, _playing ? 1 : 0, (int)_curStep, (double)_volume);
    for (int t = 0; t < NUM_TRACKS; t++) {
        char row[NUM_STEPS + 1];
        for (int s = 0; s < NUM_STEPS; s++) row[s] = getStep(t, s) ? 'x' : '.';
        row[NUM_STEPS] = 0;
        Serial.printf("%s\"%s\"", t ? "," : "", row);
    }
    Serial.print("],\"tune\":[");
    for (int t = 0; t < NUM_TRACKS; t++) Serial.printf("%s%d", t ? "," : "", (int)_tune[t]);
    Serial.print("],\"level\":[");
    for (int t = 0; t < NUM_TRACKS; t++) Serial.printf("%s%d", t ? "," : "", (int)_level[t]);
    Serial.println("]},\"source\":\"drum_synth\"}}");
}

bool DrumSynth::handleCommand(const char* cmd, const char* value) {
    if (!strcmp(cmd, "dump")) {
        // Send a full snapshot on the next pollSerial()
        _stateDirty = true;
        _stateChangedMs = 0;
        return true;
    }
    if (!strcmp(cmd, "play"))   { play(); return true; }
    if (!strcmp(cmd, "stop"))   { stop(); return true; }
    if (!strcmp(cmd, "toggle")) { togglePlay(); return true; }
    if (!strcmp(cmd, "demo"))   { loadDemoBeat(); return true; }
    if (!strcmp(cmd, "tempo"))  { setTempo(atoi(value)); return true; }
    if (!strcmp(cmd, "volume")) { setVolume(atof(value)); return true; }

    if (!strcmp(cmd, "trigger")) {
        int t = parseTrack(value);
        if (t < 0) return false;
        trigger(t);
        return true;
    }
    if (!strcmp(cmd, "clear")) {
        if (!value[0] || !strcmp(value, "all")) { clearAll(); return true; }
        int t = parseTrack(value);
        if (t < 0) return false;
        clearTrack(t);
        return true;
    }
    // "setall": six 16-char rows separated by '|' (one message, one redraw)
    if (!strcmp(cmd, "setall")) {
        const char* p = value;
        for (int t = 0; t < NUM_TRACKS && *p; t++) {
            uint16_t bits = 0;
            for (int s = 0; s < NUM_STEPS && *p && *p != '|'; s++, p++) {
                if (*p == 'x' || *p == 'X' || *p == '1') bits |= (1u << s);
            }
            while (*p && *p != '|') p++;
            if (*p == '|') p++;
            _pattern[t] = bits;
        }
        _bumpVersion();
        return true;
    }
    // "step": "track:step:on" — edit a single cell
    if (!strcmp(cmd, "step")) {
        char name[12];
        int stepIdx = -1, on = -1;
        if (sscanf(value, "%11[^:]:%d:%d", name, &stepIdx, &on) != 3) return false;
        int t = parseTrack(name);
        if (t < 0 || stepIdx < 0 || stepIdx >= NUM_STEPS) return false;
        setStep(t, stepIdx, on != 0);
        return true;
    }
    // "track:rest" commands
    const char* colon = strchr(value, ':');
    if (colon && (!strcmp(cmd, "pattern") || !strcmp(cmd, "tune") || !strcmp(cmd, "level"))) {
        char name[12];
        size_t n = min((size_t)(colon - value), sizeof(name) - 1);
        memcpy(name, value, n);
        name[n] = 0;
        int t = parseTrack(name);
        if (t < 0) return false;
        if (!strcmp(cmd, "pattern")) setPatternString(t, colon + 1);
        else if (!strcmp(cmd, "level")) setLevel(t, atoi(colon + 1));
        else setTune(t, atoi(colon + 1));
        return true;
    }
    return false;
}
