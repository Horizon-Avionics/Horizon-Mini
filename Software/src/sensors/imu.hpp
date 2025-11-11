#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include "./datalogging/data.hpp"

Adafruit_LSM6DS3TRC lsm6ds3trc;

class IMU{
    private:

    public:

        AXIS Gyro;
        AXIS Accel;
        AXIS speed;

        AXIS absGyro;

        float relAlt;

        int setup(){
            if (!lsm6ds3trc.begin_I2C()) {
                return 1;
            }
            lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
            lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
            lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_208_HZ);
            lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_208_HZ);  
            lsm6ds3trc.configInt1(false, false, true);     
            lsm6ds3trc.configInt2(false, true, false);  
            return 0;
        }
        
        int imuGetData(float deltaTime = 0.01) {
            sensors_event_t accel, gyro, temp;
            if (lsm6ds3trc.getEvent(&accel, &gyro, &temp)) {
                Accel.x = accel.acceleration.x;
                Accel.y = accel.acceleration.y;
                Accel.z = accel.acceleration.z;

                Gyro.x = gyro.gyro.x;
                Gyro.y = gyro.gyro.y;
                Gyro.z = gyro.gyro.z;

                speed.x += Accel.x * deltaTime;
                speed.y += Accel.y * deltaTime;
                speed.z += Accel.z * deltaTime;

                speed.velo = sqrt(speed.x * speed.x + speed.y * speed.y + speed.z * speed.z);

                relAlt = 3;

                return 0;
            }
            else{
                return 2;
            } 
        }
};

#endif //imu_hpp