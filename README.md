# motor_control_code

TEENSY 4.1 CODE

Edited lastly by Caleb Wilson (cwilson15@lssu.edu)

Pitfalls:
  - Left Thruster is less powerful currently (this can be changed here or the jetson propulsion_system.cpp)
  - Left thruster could also be a by-product of the potentiometer not being in good condition
  - LCD screen is not working not sure if this is from the code (switched from teensy 3.6 to teensy 4.1)
  - LCD screen could actually be damaged
  - PWM to Analog convertor is hooked up wring thanks to AMORE Team 2021-2022 (the red things)
    - need to have power 5V seperately not relying on power from the signal which they did 


RUNS ON THE ARDUINO IDE

copy the contents to edit and upload to the teensy 

!!!!!!!!!!!!!!!MAKE SURE AFTER CHANGING CODE TO ADD IT TO THIS REPOSITORY FOR THE FUTURE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

This code takes the inputs from the jetson and RC controller for the outputs to the trolling motot thrusters.

any issues concerning:
  - motor thrust
  - potentiometers on the motors
  - the relays in the GNC box
  - ACM1 dev issues
  - LCD screens

may be controlled here and is nice to check


