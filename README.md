# MTRN4230 Group 14 Project
# UR5-ROS-Gazebo 

## Required Additional Library
* `pip install opencv-python`

## Current Status
* Preliminary basic environment 1 input container, 1 output container and 10 objects
![](environment.png)
* Basic kinematics
* Raw image data

## Installation and Running
1. `git clone https://github.com/quitefrankli/MTRN4230_Group_14`
2. `cd MTRN4230_Group_14`
3. `catkin_make`
4. `source devel/setup.bash`
5. `roslaunch ur5_t2_4230 ur5_world.launch`

### For basic motion
* `python src/ur5_t2_4230/kinematics.py`
* if robot collides with something and gets stuck ---> `python src/ur5_t2_4230/testmotion.py`

### For basic vision
* `python src/ur5_t2_4230/read_image.py`
