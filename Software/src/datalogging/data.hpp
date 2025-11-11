#ifndef DATA_HPP
#define DATA_HPP

#include <Arduino.h>

struct AXIS {
    float x;
    float xFilt;
    float xOld;

    float y;
    float yFilt;
    float yOld;

    float z;
    float zFilt;
    float zOld;

    float velo;
    float veloFilt;
    float veloOld;

};

struct AIR {
    
    float temp;
    float tempFilt;
    float tempOld;

    float alt;
    float altFilt;
    float altOld;

    float pres;
    float presFilt;
    float presOld;
};

#endif //DATA_HPP
