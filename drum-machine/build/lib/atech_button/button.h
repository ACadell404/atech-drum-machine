/**
 * @file button.h
 * @brief Single Button Input Module for Atech
 *
 * Simple digital button input with edge detection.
 * Active-low (pressed = LOW) with internal pull-up.
 *
 * Atech Connector:
 * - Pin A (signal): Button
 * - Pin B (signal_b): GND (typical wiring; not driven by this driver)
 */

#ifndef BUTTON_MODULE_H
#define BUTTON_MODULE_H

#include <Arduino.h>

class ButtonModule {
public:
    ButtonModule(int pin, bool activeLow = true);

    void begin();
    bool isPressed();
    bool wasPressed();
    bool wasReleased();
    void update();
    int getState();

private:
    int _pin;
    bool _activeLow;
    bool _lastState;
    bool _currentState;
    bool _pressedFlag;
    bool _releasedFlag;
};

#endif
