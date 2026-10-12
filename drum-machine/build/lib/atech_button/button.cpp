/**
 * @file button.cpp
 * @brief Single Button Input Module Implementation
 */

#include "button.h"

ButtonModule::ButtonModule(int pin, bool activeLow)
    : _pin(pin)
    , _activeLow(activeLow)
    , _lastState(false)
    , _currentState(false)
    , _pressedFlag(false)
    , _releasedFlag(false)
{
}

void ButtonModule::begin() {
    pinMode(_pin, INPUT_PULLUP);
    _currentState = isPressed();
    _lastState = _currentState;
}

bool ButtonModule::isPressed() {
    int raw = digitalRead(_pin);
    return _activeLow ? (raw == LOW) : (raw == HIGH);
}

bool ButtonModule::wasPressed() {
    update();
    if (_pressedFlag) {
        _pressedFlag = false;
        return true;
    }
    return false;
}

bool ButtonModule::wasReleased() {
    update();
    if (_releasedFlag) {
        _releasedFlag = false;
        return true;
    }
    return false;
}

void ButtonModule::update() {
    bool reading = isPressed();
    if (reading != _currentState) {
        _currentState = reading;
        if (_currentState) {
            _pressedFlag = true;
        } else {
            _releasedFlag = true;
        }
    }
}

int ButtonModule::getState() {
    return isPressed() ? 1 : 0;
}
