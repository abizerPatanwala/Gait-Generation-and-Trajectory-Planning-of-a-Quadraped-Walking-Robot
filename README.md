# Gait-Generation-and-Trajectory-Planning-of-a-Quadraped-Walking-Robot
This project implements a forward walking wave gait on a quadrauped robot. First the forward wave gait is generated with a duty factor of 0.75. Then based on the resultant
gait phase diagram, leg trajectory planning is carried out. This involves generating joint angles for each leg to folow the gait phase diagram. The robot walks at a speed of 10cm/s with a stride length of 10 cm. The project is carried out in 3 
steps: MATLAB Simulation, Gazebo Simulation using ROS Melodic on [Dreamwalker](https://github.com/Daemiac/Dreamwalker) quaduped robot and then on hardware. Hardware implementation is caried out on a quaruped walking dog called Petoi.
## MATLAB Simulation:
For running the animation of a walking quadruped robot in MATLAB, download the files from *Gait and Trajectory generation* folder in a same folder. Then run *main.m*.
While running *main.m* run section 1, 2, 3 and 6 for animation. The animation is shown below


https://github.com/abizerPatanwala/Gait-Generation-and-Trajectory-Planning-of-a-Quadraped-Walking-Robot/assets/52460321/ec722337-b33c-4868-b98a-8fda96db4229


## Gazebo Simulation:
Put the dreamwalker in a source file of a catkin workspace. Build the workspace and source it. Download files from *ROSMatlab* folder. Open *DreamWalker_control.m*. Then on ubuntu terminal run 
*roslaunch dreamwalker_simulation dreamwalker_simulation.launch*
Once the gazebo simulations starts. Press Play and run the Matlab code *DreamWalker_control.m*. Ensure that the JointAngles.mat is in the same directory as the MATLAB code. This should enable the robot to walk.
## Hardware Implementation:
The joint angles obtained from trajectory planning were implemented on the Petoi Dog. This enabled the dog to walk using forward wave gait. The relevant files
are in the folder OpenCat
