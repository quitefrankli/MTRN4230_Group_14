#!/usr/bin/env python

import sys
from copy import deepcopy
from math import pi

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool, Header, String
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory
from ur5_t2_4230.srv import *

from ur5_gripper import trigger

# from moveit_commander import plan


class Kinematics(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        self.gripper_pub = rospy.Publisher('Gripper', Bool, queue_size=1)
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Get the name of the end-effector link
        self.end_effector_link = self.group.get_end_effector_link()

        self.group.allow_replanning(True)

        # Set the reference frame for pose targets
        reference_frame = "/world"

        # Set the ur5_arm reference frame accordingly
        self.group.set_pose_reference_frame(reference_frame)

        # We can get the name of the reference frame for this robot:
        #self.planning_frame = self.group.get_planning_frame()

        self.default_joint_states = self.group.get_current_joint_values()
        self.default_joint_states[0] = 1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        # Allow some leeway in position (meters) and orientation (radians)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_max_acceleration_scaling_factor(.15)
        self.group.set_max_velocity_scaling_factor(.25)
        self.transition_pose = deepcopy(self.default_joint_states)

        self.transition_pose[4] = -1.95

        self.go_to_idle()

    def go_to_idle(self):

        self.group.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.group.set_start_state_to_current_state()
        plan = self.group.plan()

        self.group.execute(plan, wait=True)

    def go_to_place(self):
        self.group.set_max_acceleration_scaling_factor(.05)
        self.group.set_max_velocity_scaling_factor(.15)
        self.end_joint_states = self.group.get_current_joint_values()
        self.end_joint_states[0] = 1.57691*2
        self.end_joint_states[1] = -1.71667
        self.end_joint_states[2] = 1.79266
        self.end_joint_states[3] = -1.67721
        self.end_joint_states[4] = -1.95
        self.end_joint_states[5] = 0.0

        self.group.set_joint_value_target(self.end_joint_states)

        # Set the internal state to the current state
        # self.group.set_start_state_to_current_state()
        plan = self.group.plan()

        self.group.execute(plan)

    def reset_pose(self):
        # joint_goal = [0, 0, 0, 0, 0, pi/2]
        # self.go_to_joint_state(joint_goal)
        joint_goal = [0, 0, 0, 0, 0, 0]
        self.fk(joint_goal)

    def ik(self, pose_goal):
        # Planning to a Pose Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        self.group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def fk(self, joint_goal):
        'Goes to absolute joint position using forward kinematics'

        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

    def fkr(self, joint_goal_relative):
        'Goes to relatives joint position using forward kinematics'

        joints = self.group.get_current_joint_values()
        joints += joint_goal_relative
        self.fk(joints)

        return joints

    def plan_cartesian_path(self, waypoints):
        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        ##

        # print(self.group.get_current_pose().pose)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0,
            True)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # print(plan, fraction)
        return plan, fraction

    def execute_plan(self, plan):
        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        self.group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        # END_SUB_TUTORIAL


def controller_client():
    rospy.wait_for_service('controller')
    try:
        control = rospy.ServiceProxy('controller', controller)
        resp = control()
        return resp.Xo, resp.Yo, resp.Zo
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main():
    x, y, z = controller_client()

    # create class obj
    kinematics = Kinematics()

    for i in range(len(x)):
        X = x[i]
        Y = y[i]
        Z = -0.03  # height of box relative to world
        kinematics.go_to_idle()
        rospy.sleep(1)
        start_pose = kinematics.group.get_current_pose().pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)
        waypoints = []
        wpose.position.x = X
        wpose.position.y = Y
        wpose.position.z = Z
        waypoints.append(deepcopy(wpose))
        plan, _ = kinematics.plan_cartesian_path(waypoints)
        kinematics.execute_plan(plan)
        kinematics.gripper_pub.publish(True)
        rospy.sleep(1)
        wpose.position.z += 0.07
        waypoints.append(deepcopy(wpose))
        kinematics.group.set_start_state_to_current_state()
        plan, _ = kinematics.plan_cartesian_path(waypoints)
        kinematics.execute_plan(plan)
        
        kinematics.go_to_place()
        kinematics.gripper_pub.publish(False)
        rospy.sleep(2)

    print('Done!')


if __name__ == '__main__':
    try:
        rospy.init_node('Kinematics_client', anonymous=True) 
        pub = rospy.Publisher('ui_exit', String, queue_size=1)
        main()
        rospy.sleep(5)
        pub.publish("done")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
