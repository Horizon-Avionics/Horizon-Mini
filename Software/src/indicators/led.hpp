#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>
#include <../include/constants.hpp>

Pins pin;
class R{
    public:
        void on(){
            digitalWrite(pin.LEDR, HIGH);
        }
        void off(){
            digitalWrite(pin.LEDR, LOW);
        }
};
class G{
    public:
        void on(){
            digitalWrite(pin.LEDG, HIGH);
        }
        void off(){
            digitalWrite(pin.LEDG, LOW);
        }
};
class B{
    public:
        void on(){
            digitalWrite(pin.LEDB, HIGH);
        }
        void off(){
            digitalWrite(pin.LEDB, LOW);
        }
};
class LED{
    private:
        int precmd = 0;
    public:
        R r; // led.r.on() or led.r.off()
        G g;
        B b;
        
        void ledSetup(){
            pinMode(pin.LEDR, OUTPUT);
            pinMode(pin.LEDG, OUTPUT);
            pinMode(pin.LEDB, OUTPUT);
        }


        void status(int stat){
            switch (stat){
                case 1: 
                case 2:
                case 3:
            }
        }
        void flash(int delay, int col){
            switch (col){
                case 1: 
                    r.on();
                       
            }
        }
        void ledPeriodic(int cmd) {
            if(precmd != cmd){

            }
            else{
                return;
            }
        }


};

#endif