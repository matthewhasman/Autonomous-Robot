# WALL-E: The Autonomous Robot

![2022-08-11_12-51-18_DSCF9612](https://user-images.githubusercontent.com/90156617/215256325-1261d440-5ce0-4ecf-99e1-d4bb00e4c9b6.jpg)

## Overview

WALL-E is a fully autonomous robot created over the summer of 2022. The electrical, software, control and mechanical systems were all designed from scratch. 
This robot is able to navigate a complex obstacle course. Using a PID control algorithm, WALL-E can follow a line of tape and selectively follow 
an IR beam of a particular frequency.

- Uses a STM32F103 microcontroller
- Custom PCBs for motor drivers, IR detection and filtering, reflectance sensors, hall effect sensors
- Can selectively follow an IR beam of a particular frequency, while filtering out other sources of IR radiation
- Able to follow a line of tape
- Can detect and retrieve treasures using sonar sensors
- Uses a PID control algorithm (located in main.cpp in src)

## Electrical Systems

WALL-E used a STM32F103 microcontroller called a 'BluePill'

![image](https://user-images.githubusercontent.com/90156617/215257518-310829b8-6c9a-48dc-aee1-53e703d799d9.png)

Two custom H-bridges were used to regulate the voltage across the motors and control the speed.

### H-Bridge Schematic
![image](https://user-images.githubusercontent.com/90156617/215257237-e4514968-7203-48ac-a97c-dcf0fc806a89.png)

### Completed H-Bridge
![H-bridge](https://user-images.githubusercontent.com/90156617/215257372-e1361b31-3d71-4b29-b6b2-94d62bb28196.jpg)

