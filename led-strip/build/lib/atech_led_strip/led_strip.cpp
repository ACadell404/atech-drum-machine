/**
 * @file led_strip.cpp
 * @brief Addressable LED strip effects, runtime config and serial control.
 */

#include "led_strip.h"
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

static const unsigned long FRAME_MS = 20;   // ~50 fps
static const char* PREFS_NS = "ledstrip";

static const char* ORDER_NAMES[6] = { "RGB", "RBG", "GRB", "GBR", "BRG", "BGR" };
static const neoPixelType ORDER_TYPES[6] = { NEO_RGB, NEO_RBG, NEO_GRB, NEO_GBR, NEO_BRG, NEO_BGR };
static const uint8_t DEFAULT_ORDER = 2;       // GRB

static const char* EFFECT_NAMES[LedStrip::NUM_EFFECTS] = {
    "off", "solid", "rainbow", "chase", "breathe", "sparkle", "custom", "test_colors", "test_count"
};

// ------------------------------------------------------------ lifecycle

LedStrip::LedStrip(int dataPin)
    : _pin(dataPin)
    , _strip(nullptr)
    , _buf(nullptr)
    , _count(DEFAULT_PIXELS)
    , _orderIdx(DEFAULT_ORDER)
    , _khz(800)
    , _effect(RAINBOW)
    , _r(255), _g(120), _b(40)
    , _brightness(80)
    , _speed(1.0f)
    , _flashLevel(0), _flashK(0.08f)
    , _fr(0), _fg(0), _fb(0)
    , _lastFrameMs(0)
    , _lastFlashMs(0)
    , _phase(0)
    , _testPhase(-1)
    , _rxLen(0)
    , _prefix("strip")
{
}

void LedStrip::begin() {
    _loadConfig();
    neoPixelType type = ORDER_TYPES[_orderIdx] + (_khz == 400 ? NEO_KHZ400 : NEO_KHZ800);
    _strip = new Adafruit_NeoPixel(_count, _pin, type);
    _strip->begin();
    _allocBuffer();
    _strip->clear();
    _strip->show();
}

void LedStrip::_allocBuffer() {
    free(_buf);
    _buf = (uint8_t*)calloc(_count * 3, 1);
}

void LedStrip::_loadConfig() {
    Preferences p;
    if (!p.begin(PREFS_NS, true)) return;   // nothing saved yet
    _count = constrain(p.getInt("count", DEFAULT_PIXELS), 1, MAX_PIXELS);
    _orderIdx = p.getUChar("order", DEFAULT_ORDER);
    if (_orderIdx > 5) _orderIdx = DEFAULT_ORDER;
    _khz = p.getInt("khz", 800) == 400 ? 400 : 800;
    p.end();
}

void LedStrip::_saveConfig() {
    Preferences p;
    if (!p.begin(PREFS_NS, false)) return;
    p.putInt("count", _count);
    p.putUChar("order", _orderIdx);
    p.putInt("khz", _khz);
    p.end();
}

void LedStrip::_applyType() {
    if (!_strip) return;
    _strip->updateType(ORDER_TYPES[_orderIdx] + (_khz == 400 ? NEO_KHZ400 : NEO_KHZ800));
}

// ------------------------------------------------------------ public API

void LedStrip::setEffect(Effect e) {
    if (e < 0 || e >= NUM_EFFECTS) return;
    if (e == SPARKLE && _buf) memset(_buf, 0, _count * 3);
    _effect = e;
    _testPhase = -1;
}

void LedStrip::setColor(uint8_t r, uint8_t g, uint8_t b) {
    _r = r; _g = g; _b = b;
}

void LedStrip::setBrightness(uint8_t b) { _brightness = b; }

void LedStrip::setSpeed(float s) { _speed = constrain(s, 0.1f, 5.0f); }

void LedStrip::flash(uint8_t r, uint8_t g, uint8_t b, int decayMs) {
    _fr = r; _fg = g; _fb = b;
    _flashLevel = 1.0f;
    _flashK = max(20, decayMs) / 1000.0f / 3.0f;   // time constant: ~gone after decayMs
}

void LedStrip::setPixel(int i, uint8_t r, uint8_t g, uint8_t b) {
    if (!_buf || i < 0 || i >= _count) return;
    if (_effect != CUSTOM) {
        memset(_buf, 0, _count * 3);
        _effect = CUSTOM;
    }
    _buf[i * 3] = r; _buf[i * 3 + 1] = g; _buf[i * 3 + 2] = b;
}

void LedStrip::fillRange(int from, int to, uint8_t r, uint8_t g, uint8_t b) {
    if (from > to) { int t = from; from = to; to = t; }
    for (int i = max(0, from); i <= min(to, _count - 1); i++) setPixel(i, r, g, b);
}

void LedStrip::setCount(int n) {
    n = constrain(n, 1, MAX_PIXELS);
    if (n == _count) return;
    if (_strip) {
        _strip->clear();
        _strip->show();               // blank the old length before shrinking
        _strip->updateLength(n);
    }
    _count = n;
    _allocBuffer();
    _saveConfig();
}

bool LedStrip::setOrder(const char* order) {
    char up[4] = {0};
    for (int i = 0; i < 3 && order && order[i]; i++) up[i] = toupper((unsigned char)order[i]);
    for (uint8_t i = 0; i < 6; i++) {
        if (!strcmp(up, ORDER_NAMES[i])) {
            _orderIdx = i;
            _applyType();
            _saveConfig();
            return true;
        }
    }
    return false;
}

const char* LedStrip::getOrder() const { return ORDER_NAMES[_orderIdx]; }

bool LedStrip::setKhz(int khz) {
    if (khz != 400 && khz != 800) return false;
    _khz = khz;
    _applyType();
    _saveConfig();
    return true;
}

// ------------------------------------------------------------ rendering

void LedStrip::update() {
    unsigned long now = millis();
    if (!_strip || !_buf || now - _lastFrameMs < FRAME_MS) return;
    float dt = (_lastFrameMs == 0) ? 0.0f : (now - _lastFrameMs) / 1000.0f;
    _lastFrameMs = now;

    _phase += dt * _speed;
    _render(dt);
    if (_flashLevel > 0.002f) _flashLevel *= expf(-dt / _flashK);
    else _flashLevel = 0;
    _show();
}

static inline void unpack(uint32_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = (c >> 16) & 0xFF; g = (c >> 8) & 0xFF; b = c & 0xFF;
}

void LedStrip::_render(float dt) {
    uint8_t* p = _buf;
    const int n = _count;

    switch (_effect) {
        case OFF:
            memset(p, 0, n * 3);
            break;

        case SOLID:
            for (int i = 0; i < n; i++) { p[i*3] = _r; p[i*3+1] = _g; p[i*3+2] = _b; }
            break;

        case RAINBOW:
            for (int i = 0; i < n; i++) {
                uint16_t hue = (uint16_t)((uint32_t)i * 65536UL / n + (uint32_t)(_phase * 12000.0f));
                unpack(Adafruit_NeoPixel::gamma32(Adafruit_NeoPixel::ColorHSV(hue)), p[i*3], p[i*3+1], p[i*3+2]);
            }
            break;

        case CHASE: {
            // A comet in the current colour with a fading tail, wrapping around
            float tail = max(4, n / 8);
            float head = fmodf(_phase * 20.0f, (float)n);
            for (int i = 0; i < n; i++) {
                float d = head - i;
                if (d < 0) d += n;
                float lv = d < tail ? (1.0f - d / tail) : 0.0f;
                lv *= lv;
                p[i*3] = _r * lv; p[i*3+1] = _g * lv; p[i*3+2] = _b * lv;
            }
            break;
        }

        case BREATHE: {
            float lv = 0.06f + 0.94f * (0.5f - 0.5f * cosf(_phase * 2.0f * (float)M_PI * 0.25f));
            for (int i = 0; i < n; i++) { p[i*3] = _r * lv; p[i*3+1] = _g * lv; p[i*3+2] = _b * lv; }
            break;
        }

        case SPARKLE: {
            float fade = expf(-dt * 4.0f * _speed);
            for (int i = 0; i < n * 3; i++) p[i] = p[i] * fade;
            int spawn = 1 + n / 40;
            for (int k = 0; k < spawn; k++) {
                if (random(100) < 35 * _speed) {
                    int i = random(n);
                    bool white = random(4) == 0;
                    p[i*3] = white ? 255 : _r; p[i*3+1] = white ? 255 : _g; p[i*3+2] = white ? 255 : _b;
                }
            }
            break;
        }

        case CUSTOM:
            break;   // pixels were set directly

        case TEST_COLORS: {
            int ph = (millis() / 1500) % 3;
            static const uint8_t C[3][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255} };
            for (int i = 0; i < n; i++) { p[i*3] = C[ph][0]; p[i*3+1] = C[ph][1]; p[i*3+2] = C[ph][2]; }
            if (ph != _testPhase) {
                _testPhase = ph;
                static const char* NAMES[3] = { "red", "green", "blue" };
                _emitTestPhase(NAMES[ph]);
            }
            break;
        }

        case TEST_COUNT:
            memset(p, 0, n * 3);
            for (int i = 0; i < n; i += 10) { p[i*3] = 255; p[i*3+1] = 255; p[i*3+2] = 255; }
            p[0] = 255; p[1] = 0; p[2] = 0;                                   // first: red
            p[(n-1)*3] = 0; p[(n-1)*3+1] = 255; p[(n-1)*3+2] = 0;             // last: green
            break;

        default:
            break;
    }
}

void LedStrip::_show() {
    // Test patterns ignore brightness so they are unmistakable
    bool test = (_effect == TEST_COLORS || _effect == TEST_COUNT);
    uint16_t scale = test ? 256 : (uint16_t)_brightness + 1;
    uint16_t fl = (uint16_t)(_flashLevel * scale);
    for (int i = 0; i < _count; i++) {
        uint8_t r = (_buf[i*3] * scale) >> 8;
        uint8_t g = (_buf[i*3+1] * scale) >> 8;
        uint8_t b = (_buf[i*3+2] * scale) >> 8;
        if (fl) {
            r = max(r, (uint8_t)((_fr * fl) >> 8));
            g = max(g, (uint8_t)((_fg * fl) >> 8));
            b = max(b, (uint8_t)((_fb * fl) >> 8));
        }
        _strip->setPixelColor(i, r, g, b);
    }
    _strip->show();
}

// ------------------------------------------------------------ helpers

const char* LedStrip::effectName(Effect e) {
    return (e >= 0 && e < NUM_EFFECTS) ? EFFECT_NAMES[e] : "?";
}

int LedStrip::parseEffect(const char* s) {
    if (!s) return -1;
    if (!strcmp(s, "test") || !strcmp(s, "colors")) return TEST_COLORS;
    if (!strcmp(s, "count")) return TEST_COUNT;
    for (int i = 0; i < NUM_EFFECTS; i++) {
        if (!strcmp(s, EFFECT_NAMES[i])) return i;
    }
    return -1;
}

bool LedStrip::parseColor(const char* s, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (!s || !*s) return false;
    struct Named { const char* name; uint8_t r, g, b; };
    static const Named NAMED[] = {
        {"red", 255, 0, 0}, {"green", 0, 255, 0}, {"blue", 0, 0, 255},
        {"white", 255, 255, 255}, {"warm", 255, 147, 41}, {"orange", 255, 90, 0},
        {"yellow", 255, 200, 0}, {"cyan", 0, 255, 255}, {"purple", 160, 0, 255},
        {"pink", 255, 40, 120}, {"magenta", 255, 0, 255}, {"off", 0, 0, 0}, {"black", 0, 0, 0},
    };
    for (const Named& c : NAMED) {
        if (!strcasecmp(s, c.name)) { r = c.r; g = c.g; b = c.b; return true; }
    }
    int a, b2, c;
    if (sscanf(s, "%d,%d,%d", &a, &b2, &c) == 3) {
        r = constrain(a, 0, 255); g = constrain(b2, 0, 255); b = constrain(c, 0, 255);
        return true;
    }
    const char* hex = (s[0] == '#') ? s + 1 : s;
    if (strlen(hex) == 6) {
        char* end;
        unsigned long v = strtoul(hex, &end, 16);
        if (*end == 0) { unpack((uint32_t)v, r, g, b); return true; }
    }
    return false;
}

// ------------------------------------------------------------ serial control

static bool stripJsonField(const char* line, const char* key, char* out, size_t outLen) {
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(line, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ') p++;
    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"' && n + 1 < outLen) {
            if (*p == '\\' && p[1]) p++;
            out[n++] = *p++;
        }
    } else {
        while (*p && *p != ',' && *p != '}' && n + 1 < outLen) out[n++] = *p++;
    }
    out[n] = 0;
    return true;
}

void LedStrip::pollSerial(const char* prefix) {
    _prefix = prefix;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (_rxLen < (int)sizeof(_rx) - 1) _rx[_rxLen++] = c;
            continue;
        }
        _rx[_rxLen] = 0;
        _rxLen = 0;

        char action[48], value[96];
        if (!stripJsonField(_rx, "action", action, sizeof(action))) continue;
        if (!stripJsonField(_rx, "value", value, sizeof(value))) value[0] = 0;

        size_t plen = strlen(prefix);
        if (strncmp(action, prefix, plen) != 0 || action[plen] != '_') continue;
        const char* cmd = action + plen + 1;

        if (handleCommand(cmd, value)) {
            emitState(prefix);
        } else {
            Serial.printf("{\"type\":\"log\",\"payload\":{\"message\":\"led_strip: bad command %s=%s\"}}\n", cmd, value);
        }
    }
}

bool LedStrip::handleCommand(const char* cmd, const char* value) {
    uint8_t r, g, b;

    if (!strcmp(cmd, "dump")) return true;
    if (!strcmp(cmd, "off")) { setEffect(OFF); return true; }

    if (!strcmp(cmd, "effect") || !strcmp(cmd, "test")) {
        int e = parseEffect(value);
        if (!strcmp(cmd, "test") && e != TEST_COUNT) e = TEST_COLORS;
        if (e < 0) return false;
        setEffect((Effect)e);
        return true;
    }
    if (!strcmp(cmd, "color")) {
        if (!parseColor(value, r, g, b)) return false;
        setColor(r, g, b);
        if (_effect == OFF || _effect == CUSTOM || _effect >= TEST_COLORS) setEffect(SOLID);
        return true;
    }
    if (!strcmp(cmd, "brightness")) { setBrightness(constrain(atoi(value), 0, 255)); return true; }
    if (!strcmp(cmd, "speed"))      { setSpeed(atof(value)); return true; }
    if (!strcmp(cmd, "count"))      { setCount(atoi(value)); return true; }
    if (!strcmp(cmd, "order"))      return setOrder(value);
    if (!strcmp(cmd, "khz"))        return setKhz(atoi(value));

    if (!strcmp(cmd, "flash")) {
        // "color" or "color:ms"
        char col[24];
        int ms = 250;
        const char* colon = strchr(value, ':');
        size_t n = colon ? min((size_t)(colon - value), sizeof(col) - 1) : min(strlen(value), sizeof(col) - 1);
        memcpy(col, value, n);
        col[n] = 0;
        if (colon) ms = atoi(colon + 1);
        if (!col[0]) strcpy(col, "white");
        if (!parseColor(col, r, g, b)) return false;
        flash(r, g, b, ms);
        return true;
    }
    if (!strcmp(cmd, "pixel")) {
        // "index:color"
        int idx;
        char col[24];
        if (sscanf(value, "%d:%23s", &idx, col) != 2 || !parseColor(col, r, g, b)) return false;
        setPixel(idx, r, g, b);
        return true;
    }
    if (!strcmp(cmd, "fill")) {
        // "from:to:color"
        int a, z;
        char col[24];
        if (sscanf(value, "%d:%d:%23s", &a, &z, col) != 3 || !parseColor(col, r, g, b)) return false;
        fillRange(a, z, r, g, b);
        return true;
    }
    return false;
}

void LedStrip::emitState(const char* prefix) {
    Serial.printf("{\"type\":\"event\",\"payload\":{\"event_type\":\"state\",\"key\":\"%s_state\","
                  "\"value\":{\"effect\":\"%s\",\"color\":\"#%02x%02x%02x\",\"brightness\":%d,"
                  "\"speed\":%.2f,\"count\":%d,\"order\":\"%s\",\"khz\":%d},\"source\":\"led_strip\"}}\n",
                  prefix, effectName(_effect), _r, _g, _b, _brightness, (double)_speed,
                  _count, getOrder(), _khz);
}

void LedStrip::_emitTestPhase(const char* what) {
    Serial.printf("{\"type\":\"event\",\"payload\":{\"event_type\":\"state\",\"key\":\"%s_test\","
                  "\"value\":\"%s\",\"source\":\"led_strip\"}}\n", _prefix, what);
}
