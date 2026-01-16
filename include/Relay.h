#pragma once
#include <Arduino.h>

// Uncomment to enable debug
//#define DEBUG_OUTPUT

class Relay
{
private:
    uint8_t _pin;
    bool _activeHigh; // true = HIGH is ON, false = LOW is ON

    inline void debug(const char *msg) const
    {
#ifdef DEBUG_OUTPUT
        Serial.println(msg);
#endif
    }

public:
    inline Relay(uint8_t pin, bool activeHigh)
        : _pin(pin), _activeHigh(activeHigh)
    {
    }

    inline void begin()
    {
        pinMode(_pin, OUTPUT);
        off();
        debug("Relay initialized");
    }

    inline void on()
    {
        digitalWrite(_pin, _activeHigh ? HIGH : LOW);
        debug("Relay ON");
    }

    inline void off()
    {
        digitalWrite(_pin, _activeHigh ? LOW : HIGH);
        debug("Relay OFF");
    }

    inline void toggle()
    {
        digitalWrite(_pin, !digitalRead(_pin));
        debug("Relay TOGGLE");
    }

    inline bool isOn() const
    {
        return digitalRead(_pin) == (_activeHigh ? HIGH : LOW);
    }
};