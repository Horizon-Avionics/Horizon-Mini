#ifndef NAND_HPP
#define NAND_HPP

#include <Arduino.h>

#include <SPI.h>
#include <SPIFlash.h>

#include "../include/constants.hpp"


Pins pin;

SPIFlash nand(pin.nandCS);

class NAND{

    private:

    public:

        int nandSetup(){
            if (!nand.begin()) {
                return 0;
            }
            return 1;
        }

        void writeSector(int addr, String packet){
            deleteSector(addr);
            char buf[128];
            packet.toCharArray(buf, sizeof(buf));
            nand.writeByteArray(0, (uint8_t*)buf, strlen(buf));
        }

        void deleteSector(int addr){
            nand.eraseSector(addr);
        }

};




#endif //NAND_HPP