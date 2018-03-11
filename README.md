# Humanoid
# This project is about a humanoid robot, that has two 6 dof robotic arm on a tank chassis. The project is Arduino based
# Transmitter/controller
# The controller is based on a PS2 controller. There is an arduino nano, and an LCD display on it. The reansmission is done by a HC 12 433 mhz wilreless module.
# Receiver/the robot
# The Robot contains 2 arduino nano. 
# The first one is receiving the data through HC 12 and immediatelly sending it to the second arduino via serail port.Additionally the first arduino measures the battery voltage, and lit the indicator LEDs. The chain motor controlling is also a task for the first arduino,
# The second arduino is receiving data from the first and controlls the 6 DOF arm servos.
# Wiring can be seen on fritzing. Pin layout can be different.