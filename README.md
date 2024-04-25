# Arduino Inverted Pendulum on Cart PID controller (balancing and swing-up)

This project is an implementation of an inverted pendulum system using an Arduino Mega 2560. The goal of the project is to control the pendulum to keep it upright and the cart keep its position near to the middle of the rail.
- The balancing control method is using cascade PID controller for balancing the pendulum and keep to cart at the middle of the rail at the same time. For further details about the algorithm, please visit krz4tw
https://github.com/krz4tw/Inverted-Pendulum-PID-control/?tab=readme-ov-file#inverted-pendulum-pid-control.

- I use 2 swing-up control method for swing-up, you can choose which to use
  + The first one is simple, I calculated the angular velocity of the pendulum and change the direction of the cart when the velocity reach value 0 (the highest point - the position when the peotienal energy of the pendulum is max. The PWM value I used to control the motor is constant- it could be change to fit your system.
  + ![image](https://github.com/GiaBaoM/inverted_pend/assets/161468154/59ab0dff-614e-4d24-b823-57be03569535)

  + The second one is PD cacascade controller with positive feedback for accumulating potiental energy. The outter loop input is the angle that the pendulum should reach to change the system to balance mode ( about 10=15 degree before the pendulum erects, and the output is the cart position which will be feed to the inner loop. The inner loop is to control the cart to the desire position. When the pendulum swinging, the desired cart position also changes.
![image](https://github.com/GiaBaoM/inverted_pend/assets/161468154/2e6398ce-1634-4b38-8fb6-1bdefd3ab176)

![image](https://github.com/GiaBaoM/inverted_pend/assets/161468154/dfb6b20a-53d4-46e1-9b7a-f5977196a424)

## Hardware Requirements

- Arduino Mega 2560
- 12-24V DC motor
- H-bridge Driver BTS9760
- DC-DC buck LM2596 
- DC-DC 10A Buck-Boost Constant Current Module
- Quadrature encoders for measuring the pendulum's angle and the cart 's position
- Emergency stop button
- Limit switch
- Relay
- Power source
![image](https://github.com/GiaBaoM/inverted_pend/assets/161468154/e3f413cd-1a6c-4053-b958-368b4429eeaf)

##

## Software Requirements

- PlatformIO IDE
- Arduino framework

## Library 
This project uses :
- Arduino PID Library - Version 1.2.1 by Brett Beauregard (https://github.com/br3ttb/Arduino-PID-Library)
     br3ttb@gmail.com brettbeauregard.com 
- Encoder Library - Version 1.4.4 by  Paulstoffregen (https://github.com/PaulStoffregen/Encoder) - the library is optional. In my project, i wrote a new library for handle encoder with lower resolution (using RISING/FALLING interrupt mode instead of CHANGE interrupt mode. The reason is the Atmega2560 microcontroller is not fast enough to handle high frequency encoder pulses. For example, i had a 1000 P/R encoder for cart, if the cart's encoder goes 10 round/s, the interrupt frequency is 40kHz which is too high for the micocontroller and could lead your programe to be overloaded. I think the interrupt frequency should be lower than 30kHz. Therefore, I wrote my own library to handle high speed cart's encoder. You can change the libary using the macros i defined in my library.
- PWM Library by  fizcris (https://github.com/fizcris/PWM_frequency_Arduino_change) - make the motor be quiet by using high frequency PWM instead of the default 490Hz frequency of Arduino IDE. 
## Installation

1. Clone this repository to your local machine.
2. Open the project in PlatformIO IDE.
3. Build the project using the `PlatformIO: Build` command.
   
## Contact
Github: https://github.com/GiaBaoM

Email: maigiabao73@gmail.com

Linkedin: https://www.linkedin.com/in/baomai73/

