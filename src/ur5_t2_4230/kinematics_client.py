#!/usr/bin/env python

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion, Point
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from copy import deepcopy
import numpy as np
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from tf.transformations import quaternion_from_euler
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur5_t2_4230.srv import *
# from moveit_commander import plan

class Kinematics(object):

  def __init__(self):
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    self.group = moveit_commander.MoveGroupCommander("manipulator")

    # Set the reference frame for pose targets
    reference_frame = "/base_link"

    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.group.get_planning_frame()

    # Allow replanning to increase the odds of a solution
    self.group.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.1)
    self.group.set_planning_time(0.1)
    self.group.set_max_acceleration_scaling_factor(.5)
    self.group.set_max_velocity_scaling_factor(.5)
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print("============ Printing robot state", self.robot.get_current_state())

  def reset_pose(self):
    # joint_goal = [0, 0, 0, 0, 0, pi/2]
    # self.go_to_joint_state(joint_goal)
    joint_goal = [0, 0, 0, 0, 0, 0]
    self.fk(joint_goal)

  def ik(self, pose_goal):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.group.set_planner_id("RRTConnectkConfigDefault")
    self.group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

  def fk(self, joint_goal):
    'Goes to absolute joint position using forward kinematics'
    self.group.set_planner_id("RRTConnectkConfigDefault")

    self.group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group.stop()
    
  def fkr(self, joint_goal_relative):
    'Goes to relatives joint position using forward kinematics'
    self.group.set_planner_id("RRTConnectkConfigDefault")

    joints = self.group.get_current_joint_values()
    joints += joint_goal_relative
    self.fk(joints)

    return joints

  def plan_cartesian_path(self, waypoints):
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##

    # print(self.group.get_current_pose().pose)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # print(plan, fraction)
    return plan, fraction

  def execute_plan(self, plan):
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


def controller_client():
  rospy.wait_for_service('controller')
  try:
    control = rospy.ServiceProxy('controller', controller)
    resp = control()
    print('in kinematics client')
    return resp.Xo, resp.Yo, resp.Zo
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

def main():
    rospy.init_node('MTRN4230Group14Kinematics', anonymous=True)

    x,y,z = controller_client()
    print(x)
    print(y)
    print(z)

    kinematics = Kinematics()
    kinematics.reset_pose()

    angle_up = np.array([0, -pi/3, 0, 0, 0, 0])
    bend_down = np.array([0, 0, pi/2, 0, 0, 0])
    swing_right = np.array([pi/2, 0, 0, 0, 0, 0])

    print('demonstrating kinematics...')
    rospy.sleep(3)

    # kinematics.fkr(angle_up)
    # kinematics.fkr(swing_right)
    # kinematics.fkr(bend_down)

    # kinematics.fkr(bend_down*-1)
    # kinematics.fkr(swing_right*-2)
    # kinematics.fkr(bend_down)

    # kinematics.fkr(bend_down*-1)
    # kinematics.fkr(swing_right)
    # kinematics.reset_pose()

    for i in range(len(x)):
        X = x[i]
        Y = y[i]
        Z = z
        kinematics.fk(angle_up*2)
        rospy.sleep(2)
        pos = Pose(Point(X, Y, Z), Quaternion())
        print(pos)
        #print("%s/%s" %i %len(x))
        print(i)
        kinematics.ik(pos)
        rospy.sleep(3)


    print('Done!')


    # print('demonstrating inverse kinematics...')
    # rospy.sleep(3)

    # kinematics.fk(angle_up*2)
    # pos1 = Pose(Point(0.5, 0.5, 0.5), Quaternion())
    # print(pos1)
    # kinematics.ik(pos1)
    
    # rospy.sleep(1)
    # kinematics.fk(angle_up*2)
    # pos2 = Pose(Point(0.2, -0.6, 0.3), Quaternion())
    # print(pos2)
    # kinematics.ik(pos2)

    # rospy.sleep(1)
    # kinematics.fk(angle_up*2)
    # pos3 = Pose(Point(-0.3, 0.2, 0.4), Quaternion())
    # print(pos3)
    # kinematics.ik(pos3)

    # rospy.sleep(1)
    # kinematics.fk(angle_up*2)
    # pos4 = Pose(Point(0, 0.4, 0.6), Quaternion())
    # print(pos4)
    # kinematics.ik(pos4)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    pass
