# AVC2018

This was all my work (well, almost all of it) for the 2018 Sparkfun Autonomous Vehicle Competition. I'm not including the Eagle Schematic and Board designs, nor the cad files for the various custom laser-cut components for the bot's structure.

[Some photos of the bot and the competition](doc/images)

[A post-mortem writeup of my competition experience](doc/avc-post-mortem.md)

## Bot structure

"WatBot", pronounced WattBot, is an autonomous robot. It is built on a Traxxas TRX4 chassis with a Raspberry Pi running ROS.

Peripheral elements of the robot are managed by individual nodes running on Arduinos.

1. Sonar Array - Arduino Nano
2. IMU - Arduino Zero with MPU-9250
3. Drive (for steering and esc management) - Arduino Nano
4. Odometry - Arduino Nano with dual LS7366R chips
5. Infrared - Arduino Nano with Sharp IR sensors

## Tools and resources

All PCBs, except the Arduino modules themselves, were designed in Eagle Cad and fabricated by Oshpark.

All laser cut components were designed in Libre Cad and cut by Pololu's custom laser cutting service.

I think I singlehandedly kept Sparkfun, Digikey, and Mouser in business :)
