#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>
#include "../include/constants.hpp"

class LED {
private:
    Pins pin;

    struct LEDState {
        bool active = false;
        unsigned long startT = 0;
    };

    LEDState redState, greenState, blueState;

    void handleFlash(LEDState &state, int delayMs, uint8_t ledPin) {
        if (!state.active) {
            state.active = true;
            state.startT = millis();
            digitalWrite(ledPin, HIGH);
        } else if (millis() - state.startT >= (unsigned long)delayMs) {
            digitalWrite(ledPin, LOW);
            state.active = false;
        }
    }

public:
    class R {
        LED* parent;
    public:
        R(LED* p) : parent(p) {}
        void on()  { digitalWrite(parent->pin.LEDR, HIGH); }
        void off() { digitalWrite(parent->pin.LEDR, LOW);  }
    } r{this};

    class G {
        LED* parent;
    public:
        G(LED* p) : parent(p) {}
        void on()  { digitalWrite(parent->pin.LEDG, HIGH); }
        void off() { digitalWrite(parent->pin.LEDG, LOW);  }
    } g{this};

    class B {
        LED* parent;
    public:
        B(LED* p) : parent(p) {}
        void on()  { digitalWrite(parent->pin.LEDB, HIGH); }
        void off() { digitalWrite(parent->pin.LEDB, LOW);  }
    } b{this};

    void ledSetup(Pins p) {
        pin = p;
        pinMode(pin.LEDR, OUTPUT);
        pinMode(pin.LEDG, OUTPUT);
        pinMode(pin.LEDB, OUTPUT);
    }

    void status(int stat) {
        // Example statuses:
        r.off(); g.off(); b.off();
        switch (stat) {
            case 1: g.on(); break;
            case 2: r.on(); break;
            case 3: b.on(); break;
            default: break;
        }
    }

    void flash(int delayMs, int color) {
        switch (color) {
            case 1: handleFlash(redState, delayMs, pin.LEDR); break;
            case 2: handleFlash(greenState, delayMs, pin.LEDG); break;
            case 3: handleFlash(blueState, delayMs, pin.LEDB); break;
        }
    }
};

#endif
