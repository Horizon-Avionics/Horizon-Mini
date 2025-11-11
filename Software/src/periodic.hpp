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
        void collectData(void){
            imu.imuGetData();
            bmu.bmuGetData();
            
            imuAccelX.newData = imu.accelX;
            imuAccelY.newData = imu.accelY;
            imuAccelZ.newData = imu.accelZ;

            imuGyroX.newData = imu.gyroX;
            imuGyroY.newData = imu.gyroY;
            imuGyroZ.newData = imu.gyroZ;

            imuSpeedX.newData = imu.speedX;
            imuSpeedY.newData = imu.speedY;
            imuSpeedZ.newData = imu.speedZ;

            bmuAlt.newData = bmu.Alt;
            bmuTemp.newData = bmu.Temp;

        }

        
};


#endif //periodic_hpp