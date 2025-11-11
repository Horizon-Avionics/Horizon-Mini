#include <Arduino.h>

#include <sensors/bmu.hpp>
#include <sensors/imu.hpp>

#include <datalogging/nand.hpp>
#include <datalogging/sd.hpp>

#include <indicators/buzzer.hpp>
#include <indicators/led.hpp>

#include "../include/constants.hpp"

// Workaround: some headers may define macros named 'setup' or 'loop' which
// break Arduino-style function declarations; undefine them so we can use
// the identifiers as function names.

// put function declarations here:


void setup() {
}

void loop() {
}
