#ifndef IMU_HPP
#define IMU_HPP

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

Adafruit_LSM6DS3TRC lsm6ds3trc;

class IMU{
    private:

    public:
        float imugyroX;
        float imugyroY;
        float imugyroZ;

        float absGyroX;
        float absGyroY;
        float absGyroZ;

        float imuaccelX;
        float imuaccelY;
        float imuaccelZ;

        float imuspeedX;
        float imuspeedY;
        float imuspeedZ;

        float relAlt;

        float imuvelocity;

        float autoUP;


        
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
                imuaccelX = accel.acceleration.x;
                imuaccelY = accel.acceleration.y;
                imuaccelZ = accel.acceleration.z;

                imugyroX = gyro.gyro.x;
                imugyroY = gyro.gyro.y;
                imugyroZ = gyro.gyro.z;

                imuspeedX += imuaccelX * deltaTime;
                imuspeedY += imuaccelY * deltaTime;
                imuspeedZ += imuaccelZ * deltaTime;

                imuvelocity = sqrt(imuspeedX * imuspeedX + imuspeedY * imuspeedY + imuspeedZ * imuspeedZ);
                return 0;
            }
            else{
                return 2;
            } 
        }
};

#endif //imu_hpp