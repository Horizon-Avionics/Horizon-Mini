#ifndef EMA_HPP
#define EMA_HPP

#include <Arduino.h>
#include "sensors/bmu.hpp"
#include "sensors/imu.hpp"

class EMA {
    float multFetch(int periods){
        float mult = (2/(periods + 1));
        return mult;
    }
    float filter(float emaOld, float newVal){
        float multiplier = multFetch(5);
        float ema = (newVal * multiplier) + (emaOld * (1 - multiplier));
        return ema;
    }
};

#endif //EMA.HPP