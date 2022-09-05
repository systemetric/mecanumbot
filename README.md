**Notes and code for Mr Massey's Mecanumbot**

---

The mecanumbot is a Nexus 4WD 60mm Mecanum Wheel Robot Kit.

Features: 
- four DC motors with encoders and 60mm mecanum wheels
- Nexus' Arduino Uno/Nano/?? (atmega 328p) board
- Nexus' Arduino I/O Expansion board
- *big red button*

---

Useful documents:
- [Overview PDF](docs/nexus-mecanum-robot-overview.pdf) - product overview, specifications, useful mecanum wheel diagrams. 
- [Reference PDF](docs/nexus-mecanum-robot-reference.pdf) - instructions (sometimes confusing). 

I've opted for writing code for this myself rather than using their library, since it seems a bit over the top and I actually couldn't find an official copy anywhere. 

---

Code:
- [Basic movement](src/main.cpp)
    - code moves the robot in a circle
    - comments cover the basics of wiring up the boards to control the motors

