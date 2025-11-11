#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <Arduino.h>

struct Pins {
    public:
        const int LEDR = 0;
        const int LEDG = 0;
        const int LEDB = 0;

        const int buz = 0;

        const int sdCS = 0;
        const int nandCS = 0;
};

struct Constants {
    public:
        static constexpr int emaPeriods = 5;
};

#endif