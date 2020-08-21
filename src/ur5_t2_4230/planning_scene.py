# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class Collision_Object_Adder():
  def __init__(self):
    print('in init')
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    print('after robot')
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    print('done init')
    
    
  def wait_for_box_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_plane(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.06
    box_pose.pose.orientation.w = 1.0
    box_name = 'plane'
    scene.add_box(box_name, box_pose, size=(3, 3, 0.1))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_input_container_bottom(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0.7 - 0.282575
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'input_container_bottom'
    scene.add_box(box_name, box_pose, size=(0.3175, 0.01905, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_input_container_top(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0.7 + 0.282575
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'input_container_top'
    scene.add_box(box_name, box_pose, size=(0.3175, 0.01905, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_input_container_left(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0 + 0.3175/2
    box_pose.pose.position.y = 0.7 
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'input_container_left'
    scene.add_box(box_name, box_pose, size=(0.01905, 0.5842, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_input_container_right(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0 - 0.3175/2
    box_pose.pose.position.y = 0.7 
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'input_container_right'
    scene.add_box(box_name, box_pose, size=(0.01905, 0.5842, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_output_container_bottom(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = -0.7 - 0.282575
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'output_container_bottom'
    scene.add_box(box_name, box_pose, size=(0.3175, 0.01905, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_output_container_top(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = -0.7 + 0.282575
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'output_container_top'
    scene.add_box(box_name, box_pose, size=(0.3175, 0.01905, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_output_container_left(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0 + 0.3175/2
    box_pose.pose.position.y = -0.7 
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'output_container_left'
    scene.add_box(box_name, box_pose, size=(0.01905, 0.5842, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

  def add_output_container_right(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0 - 0.3175/2
    box_pose.pose.position.y = -0.7 
    box_pose.pose.position.z = 0.127/2
    box_pose.pose.orientation.w = 1.0
    box_name = 'output_container_right'
    scene.add_box(box_name, box_pose, size=(0.01905, 0.5842, 0.127))
    self.box_name=box_name
    return self.wait_for_box_state_update(box_is_known=True, timeout=timeout)

def main():
  try:
    collision = Collision_Object_Adder()
    collision.add_input_container_left()
    collision.add_input_container_right()
    collision.add_input_container_bottom()
    collision.add_input_container_top()
    collision.add_output_container_left()
    collision.add_output_container_right()
    collision.add_output_container_bottom()
    collision.add_output_container_top()
    collision.add_plane()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
