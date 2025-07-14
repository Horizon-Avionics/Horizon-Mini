#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <Arduino.h>

struct Pins {
    public:
        static constexpr int LEDR = 0;
        static constexpr int LEDG = 0;
        static constexpr int LEDB = 0;

        static constexpr int buz = 0;

        static constexpr int sdCS = 0;
        static constexpr int nandCS = 0;
};

#endif