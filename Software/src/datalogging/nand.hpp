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

        int setup(){
            if (!nand.begin()) {
                return 0;
            }
        }

        void writeSector(int addr){

        }

        void deleteSector(int addr){
   
        }

};




#endif //NAND_HPP