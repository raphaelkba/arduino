# Arduino Mobile Robot
This repository is part of a robotics project using an arduino microcontroller to build a mobile robot. 

## Current Status
The current problem lies on the motors of the robot, which are not behaving optimaly. This raised questions as if the problem is mainly on the hardware and it has to be changed (motors, motor shield) or if it can be solved in the software level, as using encoders for controlling the motors velocity.

A first attempt to tackle this problem is to using regression to model a motor function and with a controller to keep the velocity of the motors the same and steady.

After initial tests this first solution works, however as the battery gets lower the functions for the motors do not apply correctly.
