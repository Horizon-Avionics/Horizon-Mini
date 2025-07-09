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

        float autoUP;


        
        int setup(){
            if (!lsm6ds3trc.begin_I2C()) {
                return 1;
            }
            switch (lsm6ds3trc.getAccelRange()) {
            case LSM6DS_ACCEL_RANGE_2_G:
                break;
            case LSM6DS_ACCEL_RANGE_4_G:
                break;
            case LSM6DS_ACCEL_RANGE_8_G:
                break;
            case LSM6DS_ACCEL_RANGE_16_G:
                break;
            }
            switch (lsm6ds3trc.getGyroRange()) {
            case LSM6DS_GYRO_RANGE_125_DPS:
                break;
            case LSM6DS_GYRO_RANGE_250_DPS:
                break;
            case LSM6DS_GYRO_RANGE_500_DPS:
                break;
            case LSM6DS_GYRO_RANGE_1000_DPS:
                break;
            case LSM6DS_GYRO_RANGE_2000_DPS:
                break;
            case ISM330DHCX_GYRO_RANGE_4000_DPS:
                break; // unsupported range for the DS33
            }
            switch (lsm6ds3trc.getAccelDataRate()) {
            case LSM6DS_RATE_SHUTDOWN:
                break;
            case LSM6DS_RATE_12_5_HZ:
                break;
            case LSM6DS_RATE_26_HZ:
                break;
            case LSM6DS_RATE_52_HZ:
                break;
            case LSM6DS_RATE_104_HZ:
                break;
            case LSM6DS_RATE_208_HZ:
                break;
            case LSM6DS_RATE_416_HZ:
                break;
            case LSM6DS_RATE_833_HZ:
                break;
            case LSM6DS_RATE_1_66K_HZ:
                break;
            case LSM6DS_RATE_3_33K_HZ:
                break;
            case LSM6DS_RATE_6_66K_HZ:
                break;
            }
            switch (lsm6ds3trc.getGyroDataRate()) {
            case LSM6DS_RATE_SHUTDOWN:
                break;
            case LSM6DS_RATE_12_5_HZ:
                break;
            case LSM6DS_RATE_26_HZ:
                break;
            case LSM6DS_RATE_52_HZ:
                break;
            case LSM6DS_RATE_104_HZ:
                break;
            case LSM6DS_RATE_208_HZ:
                break;
            case LSM6DS_RATE_416_HZ:
                break;
            case LSM6DS_RATE_833_HZ:
                break;
            case LSM6DS_RATE_1_66K_HZ:
                break;
            case LSM6DS_RATE_3_33K_HZ:
                break;
            case LSM6DS_RATE_6_66K_HZ:
                break;
            }
            lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
            lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
            return 0;
        }
        
        float updateIMU(){
          sensors_event_t accel;
          sensors_event_t gyro;
          sensors_event_t temp;
          lsm6ds3trc.getEvent(&accel, &gyro, &temp);

          gyroX = gyro.gyro.x;
          gyroY = gyro.gyro.y;
          gyroZ = gyro.gyro.z;

          accelX = accel.acceleration.x;
          accelY = accel.acceleration.y;
          accelZ = accel.acceleration.z;


        }
};

#endif //imu_hpp