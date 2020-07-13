#!/usr/bin/env python

# Author: Frank Li - z5115761
# Creation Date: 04/07/2020
# Last Modified: 05/07/2020
# Description:
#   Spawns objects

import tf
import rospy
# import typing
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

def test():
    print("hi")

class URDF_Object:
    orientation = Quaternion(0, 0, 0, 0)
    item_pose = Pose(Point(x=0, y=0, z=0), orientation)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    def __init__(self, node_name, object_name, point):
        '''Args: str, Point'''

        # first try to delete it (it might already exist)
        self.delete_model(node_name)

        MODEL_PATH = rospy.get_param('MODEL_PATH') + 'urdf/'

        with open(MODEL_PATH + object_name + '.urdf') as file:
            xml = file.read()
            item_pose = Pose(point, self.orientation)
            self.spawn_model(node_name, xml, '', item_pose, 'world')

class SDF_Object:
    orientation = Quaternion(0, 0, 0, 0)
    item_pose = Pose(Point(x=0, y=0, z=0), orientation)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    def __init__(self, node_name, object_name, pose):
        '''Args: str, Point'''

        # first try to delete it (it might already exist)
        self.delete_model(node_name)
        
        MODEL_PATH = rospy.get_param('MODEL_PATH') + 'sdf/'

        with open(MODEL_PATH + object_name + '.sdf') as file:
            xml = file.read()
            # item_pose = Pose(point, self.orientation)
            self.spawn_model(node_name, xml, '', pose, 'world')

def main():
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")
    print("Got it.")

    pose = Pose(Point(0.25, 0, 0.1), Quaternion(0, 1, 0, -1))
    input_container = SDF_Object('input_container', 'container', pose)
    pose = Pose(Point(0, 2, 0.1), Quaternion(0, 1, 0, -1))
    output_container = SDF_Object('output_container', 'container', pose)

    red_box = URDF_Object('red_box', 'red_box', Point(0, 0, 0.2))
    green_box = URDF_Object('green_box', 'green_box', Point(0, -0.2, 0.2))
    blue_box = URDF_Object('blue_box', 'blue_box', Point(0, 0.2, 0.2))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass