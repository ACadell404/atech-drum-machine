/**
 * @file rotary_encoder.cpp
 * @brief Rotary Encoder Knob Module Implementation
 *
 * Interrupt-driven quadrature decoding with acceleration.
 * Every edge on CLK or DT triggers an ISR, so no steps are missed
 * regardless of rotation speed or loop timing.
 *
 * Acceleration: time between steps determines a multiplier.
 * Slow turn  = 1x (fine control)
 * Fast spin  = up to 5x (quick traversal)
 *
 * Ring indicator: 6 NeoPixels in a hexagonal ring around the knob.
 * Position maps to LEDs with a smooth glow/trail effect.
 * 18 detents per revolution / 6 LEDs = 3 detents per LED.
 *
 * LED layout: 0=W, 1=SW, 2=SE, 3=E, 4=NE, 5=NW
 *
 * NOTE: All ISR code must be integer-only. No floats allowed
 * (ESP32-S3 FPU context is not saved in interrupts).
 */

#include "rotary_encoder.h"
#include <math.h>

// Debounce: ignore interrupts within this window of the last valid step
static const unsigned long DEBOUNCE_US = 3000;  // 3ms — filters contact bounce

// Acceleration thresholds (microseconds between steps)
static const unsigned long ACCEL_SLOW_US  = 120000;  // ~8 steps/sec — no acceleration
static const unsigned long ACCEL_FAST_US  =  25000;  // ~40 steps/sec — max acceleration
static const unsigned long ACCEL_RANGE_US = ACCEL_SLOW_US - ACCEL_FAST_US;

// Ring glow falloff: brightness multiplier at each LED distance from pointer.
// distance 0 = 100%, distance 1 = 35%, distance 2 = 0%
// With 12 LEDs (twice the angular resolution of the original 6), distance-1 maps
// to half the angular distance, so we boost it to 35% to keep the trail visible
// without lighting up too many pixels (which slows show() and dilutes the trail).
static const float RING_FALLOFF[] = { 1.0f, 0.35f, 0.0f };
static const int RING_FALLOFF_COUNT = 3;

// Map physical position around the ring (0=12 o'clock, going CCW) to chain index.
// The new 12-LED knob interleaves the chain like this (CCW from 12 o'clock):
//   physical pos: 0  1  2  3  4  5  6  7  8  9 10 11
//   chain idx:    0  7  1  8  2  9  3 10  4 11  5  6
// Chain indices 0-5 are the original 6 LEDs at even physical positions.
// Chain indices 6-11 are the new in-between LEDs — but the chain is wired so
// chain 6 sits between original LEDs 5 and 0 (the wrap-around at physical 11),
// chain 7 sits between original LEDs 0 and 1 (physical 1), etc.
// On older 6-LED hardware, chain indices 6-11 don't exist — data writes there
// go nowhere, so only even physical positions visibly light up.
static inline uint8_t physicalToChain(uint8_t physical) {
    if ((physical & 1) == 0) {
        // Even physical positions map directly to original LEDs 0-5
        return physical >> 1;
    }
    // Odd physical positions: chain 7 at physical 1, chain 8 at 3, ... chain 6 at 11
    // Formula: 6 + ((physical + 1) / 2) wrapped into [6, 11]
    return 6 + (((physical + 1) >> 1) % 6);
}

RotaryEncoder::RotaryEncoder(int pinClk, int pinDt, int pinSw, int pinRing)
    : _pinClk(pinClk)
    , _pinDt(pinDt)
    , _pinSw(pinSw)
    , _pinRing(pinRing)
    , _position(0)
    , _rawPosition(0)
    , _lastDirection(0)
    , _cwFlag(false)
    , _ccwFlag(false)
    , _accelEnabled(true)
    , _accelMaxMultiplier(5)
    , _lastStepTime(0)
    , _lastBtnState(false)
    , _pressedFlag(false)
    , _ring(nullptr)
    , _ringEnabled(true)
    , _ringR(255)
    , _ringG(255)
    , _ringB(255)
    , _ringOverridePos(-1.0f)
    , _lastRingPosition(INT32_MIN)
    , _ringTask(nullptr)
{
}

void RotaryEncoder::begin() {
    pinMode(_pinClk, INPUT_PULLUP);
    pinMode(_pinDt, INPUT_PULLUP);
    if (_pinSw >= 0) {
        pinMode(_pinSw, INPUT_PULLUP);
    }

    // Initialize ring if pin provided
    if (_pinRing >= 0) {
        _ring = new Adafruit_NeoPixel(RING_LEDS, _pinRing, NEO_GRB + NEO_KHZ800);
        _ring->begin();
        _ring->setBrightness(50);  // ~20% default brightness
        _ring->clear();
        _ring->show();
        Serial.print("[RotaryEncoder] Ring initialized on pin ");
        Serial.println(_pinRing);

        // Spawn background ring renderer on Core 0 — wakes from ISR for
        // sub-millisecond response, decoupled from user's main loop.
        xTaskCreatePinnedToCore(
            _ringTaskTrampoline, "RingRender", 3072, this,
            3,            // priority above default — render quickly when woken
            &_ringTask,
            0             // Core 0 (user code typically runs on Core 1)
        );
    }

    _lastStepTime = micros();

    // Only interrupt on CLK falling edge — gives exactly 1 count per detent.
    // Reading DT at that moment determines direction.
    // This avoids intermediate states between detents causing oscillation.
    attachInterruptArg(digitalPinToInterrupt(_pinClk), _isrHandler, this, FALLING);
}

void RotaryEncoder::_ringTaskTrampoline(void* param) {
    static_cast<RotaryEncoder*>(param)->_ringTaskLoop();
}

void RotaryEncoder::_ringTaskLoop() {
    while (true) {
        // Block until ISR (or user code) signals we should render
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        _updateRing();
    }
}

void RotaryEncoder::_wakeRingTask() {
    if (_ringTask) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(_ringTask, &hpw);
        if (hpw) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR RotaryEncoder::_isrHandler(void* arg) {
    RotaryEncoder* self = static_cast<RotaryEncoder*>(arg);
    self->_handleInterrupt();
}

void IRAM_ATTR RotaryEncoder::_handleInterrupt() {
    unsigned long now = micros();
    unsigned long elapsed = now - _lastStepTime;

    // Debounce: ignore bounce edges within 3ms of last valid step
    if (elapsed < DEBOUNCE_US) return;

    _lastStepTime = now;

    // On CLK falling edge: DT HIGH = clockwise, DT LOW = counter-clockwise
    int dir = digitalRead(_pinDt) ? 1 : -1;

    int multiplier = _accelEnabled ? _calcAccelMultiplier(elapsed) : 1;
    _position += dir * multiplier;
    _rawPosition += dir;  // 1:1 with physical detents, no acceleration
    _lastDirection = dir;

    if (dir > 0) _cwFlag = true;
    else _ccwFlag = true;

    // Wake the background ring renderer so the LED indicator updates within ~1ms
    // — independent of how slow the user's main loop is.
    _wakeRingTask();
}

int IRAM_ATTR RotaryEncoder::_calcAccelMultiplier(unsigned long elapsed) {
    if (elapsed >= ACCEL_SLOW_US) return 1;
    if (elapsed <= ACCEL_FAST_US) return _accelMaxMultiplier;

    // Integer-only linear interpolation (no floats allowed in ISR)
    // Maps elapsed from [ACCEL_FAST_US..ACCEL_SLOW_US] to [maxMultiplier..1]
    unsigned long fromFast = elapsed - ACCEL_FAST_US;
    int range = _accelMaxMultiplier - 1;
    return _accelMaxMultiplier - (int)(fromFast * range / ACCEL_RANGE_US);
}

void RotaryEncoder::update() {
    // Button edge detection (polling is fine for buttons)
    if (_pinSw >= 0) {
        bool currentBtn = (digitalRead(_pinSw) == LOW);  // Active low
        if (currentBtn && !_lastBtnState) {
            _pressedFlag = true;
        }
        _lastBtnState = currentBtn;
    }

    // Ring rendering happens in the background _ringTask, woken by the encoder
    // ISR and by setRing*() methods. No need to render here — keeps update()
    // fast and decouples ring response from the user's main loop frequency.
}

void RotaryEncoder::_updateRing() {
    if (!_ring || !_ringEnabled) return;

    int32_t currentPos = _rawPosition;  // Use raw (non-accelerated) position for ring

    // Skip ring update if position hasn't changed (and no override).
    // Each detent corresponds to RING_LEDS/DETENTS_PER_REV physical LEDs of
    // pointer movement. With 12 LEDs and 18 detents, that's 0.667 LEDs per
    // detent — every detent moves the pointer enough to visibly change pixels.
    if (_ringOverridePos < 0 && currentPos == _lastRingPosition) return;
    _lastRingPosition = currentPos;

    float ringPos;
    if (_ringOverridePos >= 0) {
        ringPos = _ringOverridePos;
    } else {
        // Map knob position to ring position (0.0 to RING_LEDS)
        // 18 detents per revolution, 12 LEDs → 1.5 detents per LED.
        // The ring goes counter-clockwise (physical position 0 = 12 o'clock,
        // increasing CCW around the ring), so reverse the mapping so CW knob
        // rotation = CW indicator movement (i.e. decreasing physical position).
        int32_t wrapped = currentPos % DETENTS_PER_REV;
        if (wrapped < 0) wrapped += DETENTS_PER_REV;  // Handle negative positions
        ringPos = (float)RING_LEDS - (float)wrapped * (float)RING_LEDS / (float)DETENTS_PER_REV;
    }

    _renderRing(ringPos);
}

void RotaryEncoder::_renderRing(float pos) {
    // Wrap pos into [0, RING_LEDS)
    pos = fmodf(pos, (float)RING_LEDS);
    if (pos < 0) pos += (float)RING_LEDS;

    // Build the new pixel buffer. Compare against the existing buffer so we can
    // skip the show() call entirely (which disables interrupts for ~360us) if
    // nothing actually changed.
    uint32_t newColors[RING_LEDS];
    for (int i = 0; i < RING_LEDS; i++) newColors[i] = 0;

    // Iterate over PHYSICAL positions around the ring (0 = 12 o'clock, going CCW),
    // compute brightness based on distance from pointer, then write to the
    // INTERLEAVED chain index for that physical position.
    for (int physical = 0; physical < RING_LEDS; physical++) {
        float dist = fabsf(pos - (float)physical);
        if (dist > (float)RING_LEDS / 2.0f) {
            dist = (float)RING_LEDS - dist;
        }

        float brightness = 0.0f;
        int distFloor = (int)dist;
        float distFrac = dist - (float)distFloor;

        if (distFloor < RING_FALLOFF_COUNT - 1) {
            brightness = RING_FALLOFF[distFloor] * (1.0f - distFrac)
                       + RING_FALLOFF[distFloor + 1] * distFrac;
        } else if (distFloor < RING_FALLOFF_COUNT) {
            brightness = RING_FALLOFF[distFloor] * (1.0f - distFrac);
        }

        if (brightness > 0.001f) {
            uint8_t r = (uint8_t)(_ringR * brightness);
            uint8_t g = (uint8_t)(_ringG * brightness);
            uint8_t b = (uint8_t)(_ringB * brightness);
            newColors[physicalToChain(physical)] = _ring->Color(r, g, b);
        }
    }

    // Diff against current buffer; only show() if anything changed
    bool changed = false;
    for (int i = 0; i < RING_LEDS; i++) {
        if (_ring->getPixelColor(i) != newColors[i]) {
            _ring->setPixelColor(i, newColors[i]);
            changed = true;
        }
    }
    if (changed) _ring->show();
}

int32_t RotaryEncoder::getPosition() {
    return _position;
}

void RotaryEncoder::setPosition(int32_t pos) {
    noInterrupts();
    _position = pos;
    _rawPosition = pos;
    interrupts();
}

void RotaryEncoder::resetPosition() {
    noInterrupts();
    _position = 0;
    _rawPosition = 0;
    interrupts();
}

int RotaryEncoder::getDirection() {
    int dir = _lastDirection;
    _lastDirection = 0;
    return dir;
}

bool RotaryEncoder::wasRotatedCW() {
    if (_cwFlag) {
        _cwFlag = false;
        return true;
    }
    return false;
}

bool RotaryEncoder::wasRotatedCCW() {
    if (_ccwFlag) {
        _ccwFlag = false;
        return true;
    }
    return false;
}

bool RotaryEncoder::isPressed() {
    if (_pinSw < 0) return false;
    return digitalRead(_pinSw) == LOW;
}

bool RotaryEncoder::wasPressed() {
    if (_pressedFlag) {
        _pressedFlag = false;
        return true;
    }
    return false;
}

void RotaryEncoder::setAcceleration(bool enabled, int maxMultiplier) {
    _accelEnabled = enabled;
    _accelMaxMultiplier = max(1, maxMultiplier);
}

// --- Ring methods ---

void RotaryEncoder::enableRing(bool enabled) {
    _ringEnabled = enabled;
    if (!enabled && _ring) {
        _ring->clear();
        _ring->show();
    }
    // Force re-render on next update when re-enabled
    _lastRingPosition = INT32_MIN;
    _wakeRingTask();
}

void RotaryEncoder::setRingColor(uint8_t r, uint8_t g, uint8_t b) {
    _ringR = r;
    _ringG = g;
    _ringB = b;
    _lastRingPosition = INT32_MIN;  // Force re-render
    _wakeRingTask();
}

void RotaryEncoder::setRingBrightness(uint8_t brightness) {
    if (_ring) {
        _ring->setBrightness(brightness);
        _lastRingPosition = INT32_MIN;  // Force re-render
        _wakeRingTask();
    }
}

void RotaryEncoder::setRingPosition(float pos) {
    if (pos < 0 || isnan(pos)) {
        _ringOverridePos = -1.0f;  // Re-couple to knob
    } else {
        _ringOverridePos = pos;
    }
    _lastRingPosition = INT32_MIN;  // Force re-render
    _wakeRingTask();
}
