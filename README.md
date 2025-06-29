# Horizon Avionics

## Building a fully open-source flight computer. Built by Hack Clubbers For Hack Clubbers

### By @Scooter Y, @Stars, @Meddy B, @YourLocalDndNerd

------

#### About:

The Goal of Horizon Avionics was to be able to build a full-fledged, and open-source flight computer for amateur rocketry. There is no such board online, and we are hoping for this board to streamline open-sourcing products instead of charging upwards of $400 for a ~$100 board.



#### Goals:

- Keep this board under $150 USD

- IMU/Baro

- GPS

- RF - Including a transmit tower for receiving/sending data from the rocket

- NAND + Micro SD Card Storage for logging

- 3 MCU's - MMCU, TMCU, NMCU

- H-Bridge

- Working PWM for TVC and/or Reaction Wheel

  ------

#### How it's all gonna work:

For the most part, this will all be explained over on the software repo, but a short synopsis will be provided below.

In short, we have 3 micro-controllers, the MMCU, the TMCU, and the NMCU. Each one of these MCU's is responsible for a different aspect of the functionality of the flight computer.  

- **The TMCU**

​	The TMCU is responsible for a number of different things. It takes in all the raw data from the IMU, Baro and the GPS data and sending it to the MMCU via SPI for filtering and logging. 

- **The NMCU**

  

- **The MMCU**

  

------

#### Getting Started

To view the diff changes between commits, check it out over on [CADLAB.io](https://cadlab.io/project/29344).

##### Checkout locally

1. `git clone https://github.com/Horizon-Avionics/Horizon-Avionics-Hardware`

2. Open the corresponding folder in KiCad 9.0
