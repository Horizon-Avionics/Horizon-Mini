#ifndef DATA_HPP
#define DATA_HPP

#include <Arduino.h>

struct AXIS {
    float x;
    float y;
    float z;

    float velo;
};

struct AIR {
    float temp;
    float alt;
    float pres;
};

#endif //DATA_HPP
