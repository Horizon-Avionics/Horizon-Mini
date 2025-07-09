#ifndef BMU_HPP
#define BMU_HPP

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LPS2X.h>

Adafruit_LPS22 lps;

class BMU {
    private:

    public:
        float Temp;
        float Pressure;
        float Alt;
        
        int bmuSetup() {

            if (!lps.begin_I2C()) {
                return 3;
            }
        }
        void bmuGetData() {

            sensors_event_t temp;
            sensors_event_t pressure;

            Temp = temp.temperature;
            Pressure = pressure.pressure;
            Alt = 44330.0 * (1.0 - pow(Pressure / 1013.25, 0.1903));
        }

};


#endif