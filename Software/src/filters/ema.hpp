#ifndef EMA_HPP
#define EMA_HPP

#include <Arduino.h>
#include "sensors/bmu.hpp"
#include "sensors/imu.hpp"
#include "../include/constants.hpp"

Constants constants;

class EMA {
    public:
        float multFetch(int periods){
            float mult = (2/(periods + 1));
            return mult;
        }
        float filter(float emaOld, float newVal){
            float multiplier = multFetch(constants.emaPeriods);
            float ema = (newVal * multiplier) + (emaOld * (1 - multiplier));
            return ema;
        }
};



#endif //EMA.HPP