#ifndef PERIODIC_HPP
#define PERIODIC_HPP

#include <Arduino.h>
#include "sensors/bmu.hpp"
#include "sensors/imu.hpp"
#include "filters/ema.hpp"

class sensorPeriodic {
    private:
        IMU imu;
        BMU bmu;
    public:
        void setup(void){
            imu.setup();
            bmu.bmuSetup();
            
        }

        
};


#endif //periodic_hpp