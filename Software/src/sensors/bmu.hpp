#ifndef BMU_HPP
#define BMU_HPP

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LPS2X.h>

#include "./datalogging/data.hpp"

Adafruit_LPS22 lps;

class BMU {
    private:

    public:
        AIR baro;
        
        int bmuSetup() {

            if (!lps.begin_I2C()) {
                return 3;
            }
        }
        void bmuGetData() {

            sensors_event_t temp;
            sensors_event_t pressure;

            baro.temp = temp.temperature;
            baro.pres = pressure.pressure;
            baro.alt = 44330.0 * (1.0 - pow(baro.pres / 1013.25, 0.1903));
        }

};


#endif