#ifndef EMA_HPP
#define EMA_HPP

#include <Arduino.h>
#include "sensors/bmu.hpp"
#include "sensors/imu.hpp"
#include "../include/constants.hpp"

Constants constants;

class EMA {
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

class Data {
    public:
        float oldData = 0;
        float newData = 0;

};

class data {
    private:

    public:
        Data imuGyroX;
        Data imuGyroY;
        Data imuGyroZ;

        Data imuAccelX;
        Data imuAccelY;
        Data imuAccelZ;

        Data imuSpeedX;
        Data imuSpeedY;
        Data imuSpeedZ;

        Data bmuAlt;
        Data bmuTemp;

};

#endif //EMA.HPP