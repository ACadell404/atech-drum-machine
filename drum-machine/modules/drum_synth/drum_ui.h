/**
 * @file drum_ui.h
 * @brief Step-grid screen for DrumSynth + a debounced press detector.
 *
 * Header-only and templated on the display type, so this library does not
 * depend on a particular display driver. The display must offer the
 * ST7735_TFT canvas API: fillRect / drawRect / setCursor / setTextColor /
 * setTextSize / print / display / displayRegion.
 *
 * Screen layout (160x80):
 *   y 0..11   header: selected drum + tune, tempo, PLAY/STOP (or SHIFT)
 *   y 13..78  6 rows (drums) x 16 columns (steps), 10x11 px cells
 *
 * Pushing the full frame over software SPI is slow (~270 ms), so the UI only
 * pushes the columns / header that actually changed.
 */

#ifndef DRUM_UI_H
#define DRUM_UI_H

#include <Arduino.h>

/** Debounced press-edge detector for any "is it pressed right now" input. */
struct PressEdge {
    bool raw = false;
    bool stable = false;
    unsigned long changedAt = 0;

    /** Returns true exactly once per debounced press. */
    bool pressed(bool now, unsigned long debounceMs = 15) {
        unsigned long t = millis();
        if (now != raw) {
            raw = now;
            changedAt = t;
        }
        if (t - changedAt >= debounceMs && stable != raw) {
            stable = raw;
            return stable;
        }
        return false;
    }
};

template <class TFT>
class DrumUI {
public:
    static constexpr int HEADER_H = 12;
    static constexpr int GRID_Y = 13;
    static constexpr int CELL_W = 10;
    static constexpr int CELL_H = 11;

    DrumUI(TFT& tft, DrumSynth& synth) : _tft(tft), _synth(synth) {}

    /** Draw everything once. Call after the display's begin(). */
    void begin() {
        _version = _synth.patternVersion();
        _drawAll();
        _tft.display();
    }

    // ---- editing (keeps the screen in sync without a full redraw) ----
    void setCursor(int track, int step) {
        if (track == _curTrack && step == _curStep) return;
        if (track != _curTrack) _headerDirty = true;
        _markCol(_curStep);
        _curTrack = track;
        _curStep = step;
        _markCol(_curStep);
    }

    /** Toggle the step under the cursor. Returns true if it is now on. */
    bool toggleAtCursor() {
        _synth.toggleStep(_curTrack, _curStep);
        _markCol(_curStep);
        return _synth.getStep(_curTrack, _curStep);
    }

    void setShift(bool on) {
        if (on != _shift) {
            _shift = on;
            _headerDirty = true;
        }
    }

    /**
     * Call every loop(). Moves the playhead, redraws changed regions.
     * Returns true when the sequencer advanced a step since the last call.
     */
    bool refresh() {
        bool advanced = false;

        if (_synth.patternVersion() != _version) {   // edited elsewhere (e.g. from the host)
            _version = _synth.patternVersion();
            _markChangedCols();
        }
        if (_synth.stepAdvanced()) {
            _markCol(_playStep);
            _playStep = _synth.currentStep();
            _markCol(_playStep);
            advanced = true;
        }
        if (_synth.isPlaying() != _shownPlaying || _synth.getTempo() != _shownBpm ||
            _synth.getLevel(_curTrack) != _shownLevel) {
            _headerDirty = true;
        }

        if (_fullDirty) {
            _fullDirty = false;
            _dirtyCols = 0;
            _headerDirty = false;
            _drawAll();
            _tft.display();
            return advanced;
        }

        for (int s = 0; s < DrumSynth::NUM_STEPS; s++) {
            if (_dirtyCols & (1u << s)) {
                _drawColumn(s);
                _tft.displayRegion(s * CELL_W, GRID_Y, CELL_W, DrumSynth::NUM_TRACKS * CELL_H);
            }
        }
        _dirtyCols = 0;

        // Header is the biggest region; rate-limit it while a knob is spinning
        if (_headerDirty && millis() - _lastHeaderMs >= 60) {
            _headerDirty = false;
            _lastHeaderMs = millis();
            _drawHeader();
            _tft.displayRegion(0, 0, 160, HEADER_H);
        }
        return advanced;
    }

private:
    TFT& _tft;
    DrumSynth& _synth;

    int _curTrack = 0;
    int _curStep = 0;
    int _playStep = -1;
    bool _shift = false;
    uint32_t _version = 0;

    uint16_t _dirtyCols = 0;
    bool _headerDirty = true;
    bool _fullDirty = false;
    unsigned long _lastHeaderMs = 0;

    bool _shownPlaying = false;
    int _shownBpm = -1;
    int _shownLevel = DrumSynth::MAX_LEVEL;

    static constexpr uint16_t BLACK = 0x0000;
    static constexpr uint16_t WHITE = 0xFFFF;
    static constexpr uint16_t OFF_DARK = 0x2104;   // steps in beats 1 and 3
    static constexpr uint16_t OFF_LIGHT = 0x3186;  // steps in beats 2 and 4
    static constexpr uint16_t OFF_HEAD = 0x6B4D;   // empty step under the playhead
    static constexpr uint16_t GREY = 0x8410;
    static constexpr uint16_t GREEN = 0x07E0;
    static constexpr uint16_t YELLOW = 0xFFE0;

    uint16_t _shownPattern[DrumSynth::NUM_TRACKS] = {0};

    void _markCol(int s) {
        if (s >= 0 && s < DrumSynth::NUM_STEPS) _dirtyCols |= (1u << s);
    }

    // Only repaint the columns whose steps differ from what's on screen
    void _markChangedCols() {
        for (int t = 0; t < DrumSynth::NUM_TRACKS; t++) {
            uint16_t now = _synth.getPattern(t);
            _dirtyCols |= (now ^ _shownPattern[t]);
        }
    }

    void _drawCell(int t, int s) {
        int x = s * CELL_W;
        int y = GRID_Y + t * CELL_H;
        bool on = _synth.getStep(t, s);
        bool head = (s == _playStep);
        uint16_t c;
        if (on) c = head ? WHITE : DrumSynth::trackColor565(t);
        else    c = head ? OFF_HEAD : (((s / 4) & 1) ? OFF_LIGHT : OFF_DARK);

        _tft.fillRect(x, y, CELL_W, CELL_H, BLACK);
        _tft.fillRect(x + 1, y + 1, CELL_W - 2, CELL_H - 2, c);
        if (t == _curTrack && s == _curStep) {
            _tft.drawRect(x, y, CELL_W, CELL_H, WHITE);
        }
    }

    void _drawColumn(int s) {
        for (int t = 0; t < DrumSynth::NUM_TRACKS; t++) {
            _drawCell(t, s);
            if (_synth.getStep(t, s)) _shownPattern[t] |= (1u << s);
            else                      _shownPattern[t] &= ~(1u << s);
        }
    }

    void _drawHeader() {
        _shownPlaying = _synth.isPlaying();
        _shownBpm = _synth.getTempo();
        _shownLevel = _synth.getLevel(_curTrack);

        _tft.fillRect(0, 0, 160, HEADER_H + 1, BLACK);
        _tft.setTextSize(1);

        // Selected drum, in its colour, with its velocity if below full
        _tft.setCursor(2, 2);
        _tft.setTextColor(DrumSynth::trackColor565(_curTrack));
        _tft.print(DrumSynth::trackName(_curTrack));
        if (_shownLevel < DrumSynth::MAX_LEVEL) {
            char level[6];
            snprintf(level, sizeof(level), "%d%%", _shownLevel);
            _tft.print(" ");
            _tft.print(level);
        }

        char bpm[10];
        snprintf(bpm, sizeof(bpm), "%d BPM", _shownBpm);
        _tft.setCursor(70, 2);
        _tft.setTextColor(WHITE);
        _tft.print(bpm);

        _tft.setCursor(126, 2);
        if (_shift) {
            _tft.setTextColor(YELLOW);
            _tft.print("SHIFT");
        } else if (_shownPlaying) {
            _tft.setTextColor(GREEN);
            _tft.print("PLAY");
        } else {
            _tft.setTextColor(GREY);
            _tft.print("STOP");
        }
    }

    void _drawAll() {
        _tft.fillRect(0, 0, 160, 80, BLACK);
        _drawHeader();
        for (int s = 0; s < DrumSynth::NUM_STEPS; s++) _drawColumn(s);
    }
};

#endif // DRUM_UI_H
