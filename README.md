# MTRN4230 Group 14 Project
# UR5-ROS-Gazebo 

## Required Additional Library
pip install opencv-python


## Current Status
* Preliminary basic environment 1 input container, 1 output container and 10 objects
![](environment.png)
* Basic moveit code setup. Run `$python testmoveit.py`


## ToDo:
* Convert python script into proper ROS receiver node (the current script can be *resused*)
* Camera
* UR5 Kinematics


## Installation and Running
1. `mkdir group_project & cd group_project`
2. `git clone https://github.com/quitefrankli/MTRN4230_Group_14`
3. `catkin_make`
4. `source devel/setup.bash`
5. `roslaunch ur5_t2_4230 ur5_world.launch`
6. `python src/ur5_t2_4230/spawn_objects.py`
