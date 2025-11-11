#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <Arduino.h>

struct Pins {
    public:
        int LEDR = 25;
        int LEDG = 24;
        int LEDB = 7;

        int buz = 9;

        int mosFire = A3;

        int sdCS = 11;
        int nandCS = A1;
};

struct Constants {
    public:
        static constexpr int emaPeriods = 5;
};

#endif //CONSTANTS_HPP