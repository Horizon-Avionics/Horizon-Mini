
#include <sensors/bmu.hpp>
#include <sensors/imu.hpp>

#include <datalogging/nand.hpp>
#include <datalogging/sd.hpp>

#include <indicators/buzzer.hpp>
#include <indicators/led.hpp>

#include "../include/constants.hpp"

#include "filters/filter.hpp"

// put function declarations here:

BMU bmu;
IMU imu;
LED led;

void setup() {
    filterSetup();
}

void loop() {
    updateValues();
    filterLoop();
}
