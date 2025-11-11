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
        EMA ema;
    public:
        void setup(void){
            imu.setup();
            bmu.bmuSetup();
            
        }

        void filter(){
            imu.imuGetData();
            bmu.bmuGetData();

            imu.Gyro.xOld = imu.Gyro.xFilt;
            imu.Gyro.xFilt = ema.filter(imu.Gyro.xOld, imu.Gyro.x);

            imu.Gyro.yOld = imu.Gyro.yFilt;
            imu.Gyro.yFilt = ema.filter(imu.Gyro.yOld, imu.Gyro.y);

            imu.Gyro.z = imu.Gyro.zFilt;
            imu.Gyro.zFilt = ema.filter(imu.Gyro.zOld, imu.Gyro.z);

            imu.Accel.xOld = imu.Accel.xFilt;
            imu.Accel.xFilt = ema.filter(imu.Accel.xOld, imu.Accel.x);

            imu.Accel.yOld = imu.Accel.yFilt;
            imu.Accel.yFilt = ema.filter(imu.Accel.yOld, imu.Accel.y);

            imu.Accel.zOld = imu.Accel.zFilt;
            imu.Accel.zFilt = ema.filter(imu.Accel.zOld, imu.Accel.z);

            imu.speed.xOld = imu.speed.xFilt;
            imu.speed.xFilt = ema.filter(imu.speed.xOld, imu.speed.x);

            imu.speed.yOld = imu.speed.yFilt;
            imu.speed.yFilt = ema.filter(imu.speed.yOld, imu.speed.y);

            imu.speed.zOld = imu.speed.zFilt;
            imu.speed.zFilt = ema.filter(imu.speed.zOld, imu.speed.z);
            

            imu.speed.veloOld = imu.speed.veloFilt;
            imu.speed.veloFilt = ema.filter(imu.speed.veloOld, imu.speed.velo);


            bmu.baro.altOld = bmu.baro.altFilt;
            bmu.baro.altFilt = ema.filter(bmu.baro.altOld, bmu.baro.alt);

            bmu.baro.presOld = bmu.baro.presFilt;
            bmu.baro.presFilt = ema.filter(bmu.baro.presOld, bmu.baro.pres);

            bmu.baro.tempOld = bmu.baro.tempFilt;
            bmu.baro.tempFilt = ema.filter(bmu.baro.tempOld, bmu.baro.temp);
        }
        
};


#endif //periodic_hpp