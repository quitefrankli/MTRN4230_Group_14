# MTRN4230 Group 14 Project
# UR5-ROS-Gazebo 

## Required Additional Library
* `pip install opencv-python`

## Environment
* Preliminary basic environment 1 input container, 1 output container and 14 objects\
![](environment.png)

* Interface snapshot\
![](interface.png)

* Pick and place\
![](pick_and_place.png)

## Installation and Running
1. `git clone https://github.com/quitefrankli/MTRN4230_Group_14`
2. `cd MTRN4230_Group_14`
3. `catkin_make`
4. `source devel/setup.bash`
5. `roslaunch ur5_t2_4230 ur5_world.launch` This will launch Gazebo and spawn objects
6. `roslaunch ur5_t2_4230 actions.launch` This will launch interface
