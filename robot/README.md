The robot is controlled by a rpi in the base of the robot
The host pc is connected via ethernet to the rpi
you can ssh with : ssh er@192.168.10.2
mdp : Elephant

The stepper driver of the stepper motor driving the robot axis, and its associated endstop is also connected to the rpi.

You can find a scipt for homing the axes in 
/home/er/test_home.py

and example to move the axis in : 
/home/er/test_stepper.py


and an example to move the robot + gripper in 
/home/er/test_robot.py


