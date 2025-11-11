#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DS3TRC lsm6ds3trc;

class IMU{
    private:

    public:
        float gyroX;
        float gyroY;
        float gyroZ;

        float absGyroX;
        float absGyroY;
        float absGyroZ;

        float accelX;
        float accelY;
        float accelZ;

        float speedX;
        float speedY;
        float speedZ;

        float relAlt;

        float velocity;


        
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
                accelX = accel.acceleration.x;
                accelY = accel.acceleration.y;
                accelZ = accel.acceleration.z;

                gyroX = gyro.gyro.x;
                gyroY = gyro.gyro.y;
                gyroZ = gyro.gyro.z;

                speedX += accelX * deltaTime;
                speedY += accelY * deltaTime;
                speedZ += accelZ * deltaTime;

                velocity = sqrt(speedX * speedX + speedY * speedY + speedZ * speedZ);

                relAlt = 3;

                return 0;
            }
            else{
                return 2;
            } 
        }
};

#endif //imu_hpp